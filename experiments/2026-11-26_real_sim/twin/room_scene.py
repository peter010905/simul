"""Rebuild the room shell from the reference SLAM map instead of the original exam room.

The user's description of the physical room: the two long sides are solid walls,
the two short sides are glass, and the glass is lined with chairs. The dense
SLAM points along the short sides are those chairs.

The scene keeps the 16 fitted desks and the localized marker of scene16-parity,
and replaces the original shell (walls, six columns, the south recess, windows) with:
  * long walls: per 5 cm column, the large connected SLAM wall bands are filled solid
    from the room-facing edge to the outer edge, then merged into boxes;
  * glass: collision-only panels just outside the outermost chair returns
    (no visual, so neither the GPU LiDAR nor the camera sees them; in the physical
    room the LiDAR also saw through the glass);
  * chairs: each short side is a block of plain boxes (0.6 m high) from the glass to the
    chair-cluster boundary the user drew on the board;
  * small interior objects: residual clusters of >=3 occupied cells away from desks.
No physical command is involved. The map comes from the same reference episode,
so this is a reconstruction of that room, not an independent validation of it.
"""

import argparse
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from scipy import ndimage

from build_scene import rotate

SHELL_PREFIXES = (
    "south_", "east", "north", "west", "column_", "wood_skirt_", "window",
    "end_skirt_", "repeat_supported_returns", "departure_episode_returns",
)
RES = 0.05
# Chair-cluster boundaries the user drew on the board (room frame, metres).
WEST_CHAIR_LINE = -8.75
EAST_CHAIR_LINE = [(-5.96, 7.18), (-4.71, 7.53), (-2.21, 7.42), (-0.12, 7.46),
                   (1.27, 7.67), (1.55, 7.81), (3.35, 7.53), (4.40, 7.32), (6.6, 7.32)]
CHAIR_PITCH = 0.55


def east_line(y):
    ys_, xs_ = zip(*EAST_CHAIR_LINE)
    return np.interp(y, ys_, xs_)


def chair(link, name, x, y, yaw):
    """Four legs and a seat (collision + visual), backrest away from the room."""
    c, s = math.cos(yaw), math.sin(yaw)
    def at(a, b):
        return x + c * a - s * b, y + s * a + c * b
    for i in (-1, 1):
        for j in (-1, 1):
            box(link, f"{name}_leg{i}{j}", *at(.19 * i, .19 * j), .22, .03, .03, .44,
                ".18 .18 .19 1", yaw=yaw)
    box(link, f"{name}_seat", x, y, .46, .44, .44, .04, ".25 .30 .45 1", yaw=yaw)
    box(link, f"{name}_back", *at(-.21, 0), .70, .03, .44, .44, ".25 .30 .45 1", yaw=yaw)


def occupied_world(root, fit):
    d = np.load(root / "evidence43/evidence.npz")
    grid, res, origin = d["grid"], float(d["resolution"]), d["origin"]
    r, c = np.nonzero(grid >= 65)
    metric = rotate(np.c_[c + 0.5, r + 0.5] * res, origin[2]) + origin[:2]
    return rotate(metric - fit["map_origin"], -fit["yaw"]) + fit["offset"]


def rasterize(points, low, shape):
    idx = np.floor((points - low) / RES).astype(int)
    ok = (idx >= 0).all(1) & (idx[:, 0] < shape[0]) & (idx[:, 1] < shape[1])
    img = np.zeros(shape, bool)
    img[idx[ok, 0], idx[ok, 1]] = True
    return img


def runs(mask, step):
    """Merge a boolean grid (axis0=x, axis1=y) into rectangles: y-runs per x column,
    then identical runs on consecutive columns."""
    open_ = {}
    out = []
    for i in range(mask.shape[0] + 1):
        current = set()
        if i < mask.shape[0]:
            col = np.r_[False, mask[i], False]
            edges = np.flatnonzero(np.diff(col.astype(int)))
            current = set(zip(edges[::2], edges[1::2]))
        for key in list(open_):
            if key not in current:
                out.append((open_.pop(key), i, *key))
        for key in current:
            open_.setdefault(key, i)
    return [(i0 * step, i1 * step, j0 * step, j1 * step) for i0, i1, j0, j1 in out]


def box(link, name, x, y, z, sx, sy, sz, colour=None, collide=True, yaw=0.0):
    tags = (("collision",) if collide else ()) + (("visual",) if colour else ())
    for tag in tags:
        el = ET.SubElement(link, tag, name=f"{name}_{tag}")
        ET.SubElement(el, "pose").text = f"{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}"
        ET.SubElement(ET.SubElement(ET.SubElement(el, "geometry"), "box"), "size").text = (
            f"{sx:.4f} {sy:.4f} {sz:.4f}")
        if tag == "visual":
            mat = ET.SubElement(el, "material")
            for k in ("ambient", "diffuse"):
                ET.SubElement(mat, k).text = colour


def build(root, destination, glass_west, glass_east):
    src = root / "scene16-parity"
    fit = json.loads((src / "fit.json").read_text())
    ref = json.loads((src / "reference.json").read_text())
    tables = json.loads((src / "run/simulation/demo_layout.json").read_text())["tables"]
    pts = occupied_world(root, fit)
    low = np.array([-14.0, -7.5])
    shape = (int(28.0 / RES), int(15.0 / RES))
    xs = low[0] + (np.arange(shape[0]) + 0.5) * RES
    ys = low[1] + (np.arange(shape[1]) + 0.5) * RES
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    occ = rasterize(pts, low, shape)
    inside_glass = (X > glass_west) & (X < glass_east)
    occ &= inside_glass

    # Desk footprints (+12 cm): legs are modelled by the desk models themselves.
    desk = np.zeros(shape, bool)
    for t in tables:
        local = rotate(np.c_[X.ravel() - t["x"], Y.ravel() - t["y"]], t["yaw"])
        desk |= ((abs(local[:, 0]) <= t["sx"] / 2 + 0.12)
                 & (abs(local[:, 1]) <= t["sy"] / 2 + 0.12)).reshape(shape)
    occ &= ~desk

    # Long walls: large 8-connected components that reach the north or south band.
    labels, n = ndimage.label(occ, np.ones((3, 3)))
    sizes = ndimage.sum(occ, labels, range(1, n + 1))
    wall = np.zeros(shape, bool)
    for k in range(1, n + 1):
        part = labels == k
        if sizes[k - 1] >= 150 and (Y[part].max() > 5.0 or Y[part].min() < -5.3):
            wall |= part
    # Only the long-side bands are wall; vertical parts of the same components along
    # the short sides are chairs packed against the glass.
    wall &= (Y > 4.5) | (Y < -5.0)
    north = np.zeros(shape, bool)
    south = np.zeros(shape, bool)
    # The SLAM band edge carries cell-level noise; the physical walls are straight.
    # A 0.55 m running median of the room-facing edge removes the noise and keeps
    # structures wider than about 0.5 m (the column and the partition steps).
    for half_sign, target in ((1, north), (-1, south)):
        inner = np.full(shape[0], np.nan)
        outer = np.full(shape[0], np.nan)
        for i in range(shape[0]):
            j = np.flatnonzero(wall[i] & ((Y[i] > 0) if half_sign > 0 else (Y[i] < 0)))
            if len(j):
                inner[i], outer[i] = (j.min(), j.max()) if half_sign > 0 else (j.max(), j.min())
        valid = np.isfinite(inner)
        smooth = inner.copy()
        idx = np.flatnonzero(valid)
        for i in idx:
            window = inner[max(0, i - 5):i + 6]
            smooth[i] = np.round(np.median(window[np.isfinite(window)]))
        # A 10 cm face grid and a fixed 0.35 m thickness let neighbouring columns merge
        # into long boxes (fewer collision shapes keep the simulator near real time).
        for i in idx:
            face = int(round(smooth[i] / 2.) * 2)
            a_, b_ = (face, face + 7) if half_sign > 0 else (face - 7, face)
            target[i, max(a_, 0):min(b_, shape[1] - 1) + 1] = True
    # Close one-column gaps so the LiDAR cannot see through merged wall boxes.
    north = ndimage.binary_closing(north, np.ones((3, 1))) & (Y > 0)
    south = ndimage.binary_closing(south, np.ones((3, 1))) & (Y < 0)
    walls = north | south
    rest = occ & ~ndimage.binary_dilation(walls, iterations=1)

    # Chairs along the glass: short-side bands between the long walls.
    def wall_y(mask, x, top):
        i = np.clip(int((x - low[0]) / RES), 0, shape[0] - 1)
        j = np.flatnonzero(mask[i])
        return ys[j.min()] if top and len(j) else ys[j.max()] if len(j) else (5.6 if top else -5.6)

    chair_zone = (X < WEST_CHAIR_LINE) | (X > east_line(Y))
    chairs = rest & chair_zone
    interior = rest & ~chair_zone
    labels, n = ndimage.label(interior, np.ones((3, 3)))
    sizes = ndimage.sum(interior, labels, range(1, n + 1))
    keep = np.isin(labels, 1 + np.flatnonzero(sizes >= 3))
    interior = interior & keep

    start = np.array(ref["start"][:2])
    clear = np.hypot(X - start[0], Y - start[1]) >= 0.45

    sdf = ET.parse(src / "world/osp_studio.sdf")
    world = sdf.find("world")
    removed = []
    for m in list(world.findall("model")):
        if m.get("name", "").startswith(SHELL_PREFIXES):
            removed.append(m.get("name"))
            world.remove(m)
    model = ET.SubElement(world, "model", name="slam_room")
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="shell")
    summary = dict(removed_shell_models=len(removed))

    rects = runs(walls & clear, RES)
    for k, (x0, x1, y0, y1) in enumerate(rects):
        box(link, f"wall_{k}", low[0] + (x0 + x1) / 2, low[1] + (y0 + y1) / 2, 1.25,
            x1 - x0, y1 - y0, 2.5, ".90 .89 .86 1")
    summary["wall_boxes"] = len(rects)

    # Glass panels span between the inner faces of the long walls (collision only).
    for side, x in (("west", glass_west), ("east", glass_east)):
        # Overlaps the long walls at both ends so there is no gap at the corners.
        top, bottom = 6.6, -6.8
        box(link, f"glass_{side}", x, (top + bottom) / 2, 1.25, 0.04, top - bottom, 2.5)
        summary[f"glass_{side}"] = dict(x=x, y=[float(bottom), float(top)])
    # Collision-only outer caps behind the long walls close the glass/wall corners.
    for side, y in (("north", 6.75), ("south", -6.95)):
        box(link, f"glass_cap_{side}", (glass_west + glass_east) / 2, y, 1.25,
            glass_east - glass_west + 0.2, 0.1, 2.5)

    def posts(mask, name, height, colour):
        step = 0.08
        idx = np.argwhere(mask & clear)
        pts_ = low + (idx + 0.5) * RES
        cells = np.unique(np.floor((pts_ - low) / step).astype(int), axis=0)
        cmask = np.zeros((int(28 / step) + 1, int(15 / step) + 1), bool)
        cmask[cells[:, 0], cells[:, 1]] = True
        rs = runs(cmask, step)
        for k, (x0, x1, y0, y1) in enumerate(rs):
            box(link, f"{name}_{k}", low[0] + (x0 + x1) / 2, low[1] + (y0 + y1) / 2,
                height / 2, x1 - x0, y1 - y0, height, colour)
        return cmask, step, len(rs)

    # Each chair cluster is one block of boxes from the glass to the drawn boundary,
    # in 0.25 m bands along y that stop at the long-wall faces.
    placed = []
    band = 0.25
    for side in ("west", "east"):
        for y0 in np.arange(-6.5, 6.5, band):
            cy = y0 + band / 2
            if side == "west":
                x0, x1 = glass_west + .02, WEST_CHAIR_LINE
            else:
                x0, x1 = float(east_line(cy)), glass_east - .02
            xc = (x0 + x1) / 2
            if not wall_y(south, xc, False) < cy < wall_y(north, xc, True):
                continue
            box(link, f"chair_block_{len(placed)}", xc, cy, .30, x1 - x0, band, .60, ".40 .42 .50 1")
            placed.append(dict(x=float(xc), y=float(cy), sx=float(x1 - x0), sy=band, yaw=0.0, side=side))
    summary["chairs"] = len(placed)
    summary["chair_lines"] = dict(west=WEST_CHAIR_LINE, east=EAST_CHAIR_LINE)
    _, _, summary["interior_posts"] = posts(interior, "object", 0.30, ".20 .20 .21 1")

    # Snap the localized print onto the nearest wall face, facing the room.
    targets = json.loads((src / "scene.json").read_text())["targets"]
    for t in targets:
        tm = world.find(f"model[@name='target_{t['id']}']")
        face = wall_y(south if t["y"] < 0 else north, t["x"], t["y"] > 0)
        if abs(face - t["y"]) <= 0.6:
            offset = (-RES / 2 - 0.004) if t["y"] > 0 else (RES / 2 + 0.004)
            t.update(y=float(face + offset), yaw=-math.pi / 2 if t["y"] > 0 else math.pi / 2,
                     attached_to="SLAM estimate snapped to reconstructed wall face")
            tm.find("pose").text = f"{t['x']} {t['y']} {t['z']} 0 0 {t['yaw']}"
    summary["targets"] = targets

    shutil.copytree(src, destination)
    sdf.write(destination / "world/osp_studio.sdf", encoding="utf-8", xml_declaration=True)
    scene = json.loads((src / "scene.json").read_text())
    scene.update(targets=targets, partitions=[], south_recess=None,
                 name="실물 SLAM 벽·유리·의자 재구성",
                 shell="Long walls and chairs from reference SLAM; short sides are glass lined with chairs.")
    for p in (destination / "scene.json", destination / "run/simulation/scene.json"):
        p.write_text(json.dumps(scene))
    import yaml
    doc = yaml.safe_load((src / "scene.yaml").read_text())
    doc["targets"] = targets
    (destination / "scene.yaml").write_text(yaml.safe_dump(json.loads(json.dumps(doc))))
    layer = dict(walls=[dict(x=low[0] + (a + b) / 2, y=low[1] + (c + d) / 2, sx=b - a, sy=d - c)
                        for a, b, c, d in rects],
                 glass=[summary["glass_west"], summary["glass_east"]], chairs=placed)
    (destination / "room_shell.json").write_text(json.dumps(layer))
    (destination / "room_support.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps({k: v for k, v in summary.items() if k != "targets"}))


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("root", type=Path)
    p.add_argument("destination", type=Path)
    p.add_argument("--glass-west", type=float, default=-9.80)
    p.add_argument("--glass-east", type=float, default=10.30)
    a = p.parse_args()
    build(a.root, a.destination, a.glass_west, a.glass_east)
