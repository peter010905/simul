"""Seeded random movable objects and markers inside the SLAM-reconstructed room (scene16-room).

Long walls get one or two random protrusions each (pillars, radiators, cabinets); small floor
debris (low pieces under the LiDAR plane, bottles, bags) is scattered. Desks move by up to 0.15 m and 3 degrees (the joined middle rows move as groups), with
leg spacing, leg size and top size varied per seed. Objects are boxes only (sizes and heights as in the team exam generator), three by default:
one wedged between desks in the middle of the room (within 0.45 m of two desks), one next
to a desk (5-30 cm from its legs), and one more from those classes, a long wall or open
floor. One print is always on the box between desks. Three ArUco prints as in the exam layout: two on box faces and one on a
long wall, centre height 0.25-0.40 m. The short sides are glass lined with chairs, so wall prints go
on the two long walls only. The SLAM interior returns of the reference day (that day's
boxes) are removed; fixed structures near the long walls are kept. The robot start stays
the physical reference start. The chair clusters along both glass ends are sampled
per seed (count via fill ratio, depth, pitch, ragged edge, a few strays) as chairs with
3 cm legs, so the LiDAR sees scattered leg returns as in the physical room. They are
not worth exploring, so prints and movable bodies stay at least 1 m from any chair
and from the chair-cluster boundary the user drew. Every print must have a free, start-connected standpoint
0.8-1.6 m in front of it with a clear sight line. Ground truth goes only to the evaluator
(scene.yaml / scene.json); nothing is written into a policy workspace.
"""

import argparse
import copy
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy import ndimage

from room_scene import WEST_CHAIR_LINE, east_line

RES = 0.05
LOW = np.array([-14.0, -7.5])
SHAPE = (int(28.0 / RES), int(15.0 / RES))
ROBOT_RADIUS = 0.22
PRINT_SIZE = 0.2657523127791951  # measured print edge in the reference episode
CHAIR_MARGIN = 1.0  # chair clusters are not worth exploring: keep prints and bodies 1 m away


_CHAIR_CLEARANCE = None  # metres from the nearest sampled chair, set per layout


def clear_of_chairs(points):
    pts = np.atleast_2d(points)
    # The drawn chair-cluster zone carries no prints even where a seed placed few chairs.
    if (np.any(pts[:, 0] < WEST_CHAIR_LINE + CHAIR_MARGIN)
            or np.any(pts[:, 0] > east_line(pts[:, 1]) - CHAIR_MARGIN)):
        return False
    for p in pts:
        c = cell(p)
        if not (0 <= c[0] < SHAPE[0] and 0 <= c[1] < SHAPE[1]) or _CHAIR_CLEARANCE[c] < CHAIR_MARGIN:
            return False
    return True


def chair_model(link, name, x, y, yaw):
    """One collision box; four 3 cm legs and a seat as visuals. LiDAR sees only the legs."""
    c, s = math.cos(yaw), math.sin(yaw)

    # One collision box per chair (the robot cannot pass under or between its legs);
    # legs and seat are visuals, so the GPU LiDAR and the camera still see a chair.
    col = ET.SubElement(link, "collision", name=f"{name}_body_collision")
    ET.SubElement(col, "pose").text = f"{x:.4f} {y:.4f} .23 0 0 {yaw:.4f}"
    ET.SubElement(ET.SubElement(ET.SubElement(col, "geometry"), "box"), "size").text = ".44 .44 .46"

    def part(tag_name, dx, dy, z, size, colour, collide=False):
        px, py = x + c * dx - s * dy, y + s * dx + c * dy
        for tag in (("collision", "visual") if collide else ("visual",)):
            el = ET.SubElement(link, tag, name=f"{name}_{tag_name}_{tag}")
            ET.SubElement(el, "pose").text = f"{px:.4f} {py:.4f} {z:.4f} 0 0 {yaw:.4f}"
            ET.SubElement(ET.SubElement(ET.SubElement(el, "geometry"), "box"), "size").text = size
            if tag == "visual":
                mat = ET.SubElement(el, "material")
                for k in ("ambient", "diffuse"):
                    ET.SubElement(mat, k).text = colour

    for i in (-1, 1):
        for j in (-1, 1):
            part(f"leg{i}{j}", .19 * i, .19 * j, .22, ".03 .03 .44", ".18 .18 .19 1")
    part("seat", 0, 0, .46, ".44 .44 .04", ".25 .30 .45 1")



def sample_chairs(world, walls, rng):
    """Chair clusters on both glass ends with a random count, depth, pitch and ragged edge."""
    link = world.find("model[@name='slam_room']/link")
    for el in list(link):
        if el.get("name", "").startswith(("chair_block", "chair_")):
            link.remove(el)
    model = ET.SubElement(world, "model", name="chair_clusters")
    ET.SubElement(model, "static").text = "true"
    clink = ET.SubElement(model, "link", name="chairs")
    glass = {"west": -9.80, "east": 10.30}
    params, chairs = {}, []

    def inside(x, y):
        # between the two long-wall faces (a chair behind a wall corner is outside the room)
        north, south = wall_face(walls, x, True), wall_face(walls, x, False)
        return north is not None and south is not None and south + .3 < y < north - .3
    for side in ("west", "east"):
        depth = float(rng.uniform(.8, 2.0) if side == "west" else rng.uniform(1.4, 3.0))
        pitch = float(rng.uniform(.50, .66))
        fill_p = float(rng.uniform(.70, .97))
        rough = float(rng.uniform(.10, .55))
        strays = int(rng.integers(0, 6))
        params[side] = dict(depth_m=round(depth, 2), pitch_m=round(pitch, 2), fill=round(fill_p, 2),
                            ragged_m=round(rough, 2), strays=strays)
        inward = 1. if side == "west" else -1.
        edge = []
        offset = 0.
        for y in np.arange(-6.2, 6.4, pitch):
            # ragged edge: a bounded random walk of the row depth
            offset = float(np.clip(offset + rng.normal(0, rough * .6), -rough, rough))
            d = max(.45, depth + offset)
            row_max = None
            for k in np.arange(.30, d, pitch):
                if rng.random() > fill_p:
                    continue
                cx = glass[side] + inward * k + rng.uniform(-.05, .05)
                cy = y + rng.uniform(-.05, .05)
                i0, j0 = cell((cx - .3, cy - .3))
                i1, j1 = cell((cx + .3, cy + .3))
                if walls[max(i0, 0):i1 + 1, max(j0, 0):j1 + 1].any() or not inside(cx, cy):
                    continue
                chairs.append((cx, cy, (0. if side == "west" else math.pi) + rng.uniform(-.5, .5)))
                row_max = k
            if row_max is not None:
                edge.append((float(y), float(glass[side] + inward * row_max)))
        for _ in range(strays):
            if not edge:
                break
            y, x = edge[rng.integers(len(edge))]
            cx, cy = x + inward * rng.uniform(.4, .9), y + rng.uniform(-.2, .2)
            i0, j0 = cell((cx - .3, cy - .3))
            i1, j1 = cell((cx + .3, cy + .3))
            if not walls[max(i0, 0):i1 + 1, max(j0, 0):j1 + 1].any() and inside(cx, cy):
                chairs.append((cx, cy, rng.uniform(-math.pi, math.pi)))
        params[side]["edge"] = edge
    for k, (x, y, yaw) in enumerate(chairs):
        chair_model(clink, f"chair_{k}", x, y, yaw)
    params["count"] = len(chairs)
    return params, chairs


def corners(body, margin=0.0):
    c, s = math.cos(body["yaw"]), math.sin(body["yaw"])
    hx, hy = body["sx"] / 2 + margin, body["sy"] / 2 + margin
    return np.array([[body["x"] + c * a - s * b, body["y"] + s * a + c * b]
                     for a, b in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))])


def fill(mask, poly):
    # Grid rows are x cells and columns are y cells; OpenCV points are (column, row).
    pix = np.round((poly - LOW) / RES - .5).astype(np.int32)[:, ::-1]
    img = np.zeros(mask.shape, np.uint8)
    cv2.fillPoly(img, [pix], 1)
    mask |= img.astype(bool)


def static_occupancy(world, layout, prefix=None):
    occ = np.zeros(SHAPE, bool)
    for model in world.findall("model"):
        mp = np.fromstring(model.findtext("pose") or "0 0 0 0 0 0", sep=" ")
        if model.get("name") in ("floor",):
            continue
        for col in model.iter("collision"):
            size = col.findtext("geometry/box/size")
            if not size or (prefix and not col.get("name", "").startswith(prefix)):
                continue
            sx, sy, sz = np.fromstring(size, sep=" ")
            p = np.fromstring(col.findtext("pose") or "0 0 0 0 0 0", sep=" ")
            z = mp[2] + p[2]
            if z - sz / 2 > 0.6 or z + sz / 2 < 0.02:  # desk tops are handled by footprints
                continue
            fill(occ, corners(dict(x=mp[0] + p[0], y=mp[1] + p[1], yaw=mp[5] + p[5], sx=sx, sy=sy)))
    for t in ([] if prefix else layout):
        fill(occ, corners(t))
    return occ


def reachable(occ, start):
    free = ~ndimage.binary_dilation(occ, iterations=int(math.ceil(ROBOT_RADIUS / RES)))
    labels, _ = ndimage.label(free)
    i, j = np.floor((np.asarray(start) - LOW) / RES).astype(int)
    return labels == labels[i, j] if labels[i, j] else np.zeros_like(free)


def cell(p):
    return tuple(np.floor((np.asarray(p) - LOW) / RES).astype(int))


def sight(occ, a, b):
    n = max(2, int(math.dist(a, b) / (RES / 2)))
    for k in range(1, n - 1):
        p = np.asarray(a) + (np.asarray(b) - a) * k / n
        if occ[cell(p)]:
            return False
    return True


def draw_obstacle(index, rng):
    host = f"movable_{index + 1}"
    kind = str(rng.choice(("box", "box", "box", "screen", "elbow")))
    if kind == "box":
        sx, sy = rng.uniform(.46, 1.25), rng.uniform(.46, 1.1)
    elif kind == "screen":
        sx, sy = rng.uniform(.85, 2.1), rng.uniform(.16, .32)
    else:
        sx, sy = rng.uniform(.95, 1.75), rng.uniform(.95, 1.55)
    return dict(name=host, object_id=host, shape=kind, sx=float(sx), sy=float(sy),
                height=float(rng.uniform(.55, .95)))


def pieces(frame, rng):
    if frame["shape"] != "elbow":
        return [frame]
    thickness = float(rng.uniform(.22, .36))
    c, s = math.cos(frame["yaw"]), math.sin(frame["yaw"])
    sx, sy = frame["sx"], frame["sy"]
    out = []
    for suffix, lx, ly, width, depth in (("a", 0., (thickness - sy) / 2, sx, thickness),
                                         ("b", (thickness - sx) / 2, thickness / 2, thickness, sy - thickness)):
        out.append(dict(frame, name=frame["name"] + "_" + suffix, x=frame["x"] + c * lx - s * ly,
                        y=frame["y"] + s * lx + c * ly, sx=float(width), sy=float(depth)))
    return out


def rect_distance(point, t):
    """Distance from a point to a desk footprint."""
    c, s = math.cos(t["yaw"]), math.sin(t["yaw"])
    dx, dy = point[0] - t["x"], point[1] - t["y"]
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    return math.hypot(max(abs(lx) - t["sx"] / 2, 0.), max(abs(ly) - t["sy"] / 2, 0.))


def place_box(frame, mode, rng, layout, walls):
    """Candidate pose for a box in one placement class; returns (body, collision margin)."""
    if mode == "between":
        # Wedged between desks in the middle of the room: within 0.45 m of two different
        # desks (their legs on both sides), where the physical robot struggled most.
        body = dict(frame, sx=float(rng.uniform(.46, .80)), sy=float(rng.uniform(.46, .80)),
                    x=float(rng.uniform(-7.2, 5.3)), y=float(rng.uniform(-3.4, 3.4)),
                    yaw=float(rng.uniform(-math.pi, math.pi)))
        pts = list(corners(body)) + [np.array([body["x"], body["y"]])]
        near = sum(1 for t in layout if min(rect_distance(q, t) for q in pts) <= .45)
        return (body, .03) if near >= 2 else (None, 0)
    if mode == "desk":
        t = layout[rng.integers(len(layout))]
        hx, hy = t["sx"] / 2, t["sy"] / 2
        turn = float(rng.choice([0., math.pi / 2]))
        yaw = t["yaw"] + turn + float(rng.uniform(-.15, .15))
        bx, by = (frame["sx"] / 2, frame["sy"] / 2) if turn == 0 else (frame["sy"] / 2, frame["sx"] / 2)
        gap = float(rng.uniform(.05, .30))
        side = int(rng.integers(4))
        if side < 2:   # at an end of the desk, beside its legs
            lx, ly = (1 if side == 0 else -1) * (hx + gap + bx), float(rng.uniform(-hy, hy))
        else:          # along a long side
            lx, ly = float(rng.uniform(-hx, hx)), (1 if side == 2 else -1) * (hy + gap + by)
        c, s = math.cos(t["yaw"]), math.sin(t["yaw"])
        return dict(frame, x=t["x"] + c * lx - s * ly, y=t["y"] + s * lx + c * ly, yaw=yaw), .03
    if mode == "wall":
        top = bool(rng.integers(2))
        x = float(rng.uniform(-7.5, 6.5))
        face = wall_face(walls, x, top)
        if face is None:
            return None, 0
        yaw = float(rng.choice([0., math.pi / 2])) + float(rng.uniform(-.1, .1))
        half = (frame["sy"] if abs(math.sin(yaw)) < .5 else frame["sx"]) / 2
        gap = float(rng.uniform(.10, .40))
        y = face - (gap + half) if top else face + gap + half
        return dict(frame, x=x, y=y, yaw=yaw), .05
    return dict(frame, x=float(rng.uniform(-7.5, 6.5)), y=float(rng.uniform(-4.8, 4.8)),
                yaw=float(rng.uniform(-math.pi, math.pi))), .06


def wall_face(occ, x, top):
    """Room-facing wall face y at x on the north (top) or south long wall."""
    i = int((x - LOW[0]) / RES)
    js = np.arange(int((0 - LOW[1]) / RES), SHAPE[1]) if top else np.arange(int((0 - LOW[1]) / RES), -1, -1)
    hit = [j for j in js if occ[i, j] and abs(LOW[1] + (j + .5) * RES) > 4.3]
    if not hit:
        return None
    j = hit[0]
    return LOW[1] + (j if top else j + 1) * RES


def sample_clutter_chairs(world, walls, layout, rng, start):
    """Extra chairs that are not glass-end clusters: rows along the long walls and chairs
    pushed against desks, to test that the chair-wall logic does not overfit."""
    model = world.find("model[@name='chair_clusters']")
    link = model.find("link")
    placed = []

    def free(x, y):
        i0, j0 = cell((x - .3, y - .3))
        i1, j1 = cell((x + .3, y + .3))
        return (not walls[max(i0, 0):i1 + 1, max(j0, 0):j1 + 1].any()
                and math.dist((x, y), start) > 1.2
                and all(math.dist((x, y), q[:2]) > .5 for q in placed)
                and all(rect_distance((x, y), t) > .2 for t in layout))

    # rows along the long walls (chairs pushed back against the wall)
    for top in (True, False):
        for _ in range(int(rng.integers(2, 5))):
            x0 = float(rng.uniform(-7.5, 5.5))
            length = float(rng.uniform(1.5, 4.0))
            pitch = float(rng.uniform(.55, .75))
            for x in np.arange(x0, min(x0 + length, 6.5), pitch):
                face = wall_face(walls, x, top)
                if face is None:
                    continue
                y = face - (.28 + rng.uniform(0, .2)) if top else face + .28 + rng.uniform(0, .2)
                if free(x, y):
                    yaw = (-math.pi / 2 if top else math.pi / 2) + rng.uniform(-.4, .4)
                    placed.append((x, y, yaw, "wall"))
    # chairs at desks (pulled up to a long side, as in a classroom)
    for t in layout:
        if rng.random() > .6:
            continue
        c, s_ = math.cos(t["yaw"]), math.sin(t["yaw"])
        for _ in range(int(rng.integers(1, 5))):
            side = 1 if rng.random() < .5 else -1
            lx = float(rng.uniform(-t["sx"] / 2 + .25, t["sx"] / 2 - .25))
            ly = side * (t["sy"] / 2 + .25 + rng.uniform(0, .15))
            x, y = t["x"] + c * lx - s_ * ly, t["y"] + s_ * lx + c * ly
            if free(x, y):
                placed.append((x, y, t["yaw"] + (math.pi / 2 if side < 0 else -math.pi / 2) + rng.uniform(-.4, .4), "desk"))
    base = len(list(link.iter("collision")))
    for k, (x, y, yaw, _) in enumerate(placed):
        chair_model(link, f"chair_x{k}", x, y, yaw)
    return dict(wall=sum(1 for p_ in placed if p_[3] == "wall"), desk=sum(1 for p_ in placed if p_[3] == "desk"))


def perturb_desks(world, layout, fit, rng):
    """Move, rotate and resize every desk a little (legs included) so a policy cannot
    rely on one exact desk geometry. Rows of three joined middle desks move together."""
    link = world.find("model[@name='demo_furniture']/link")
    long0, short0 = fit["leg_dimensions"]
    group_shift = {}
    new_layout = []
    for t in layout:
        t = dict(t)
        if t["name"].startswith("table_middle_"):
            k = int(t["name"].rsplit("_", 1)[1])
            g = 0 if k <= 3 else 1
            if g not in group_shift:
                group_shift[g] = (rng.uniform(-.10, .10), rng.uniform(-.12, .12))
            dx, dy = group_shift[g]
            dx += rng.uniform(-.03, .03)
            dyaw = rng.uniform(-.01, .01)
        else:
            dx, dy, dyaw = rng.uniform(-.15, .15), rng.uniform(-.15, .15), rng.uniform(-.05, .05)
        long_ = long0 + rng.uniform(-.06, .06)
        short_ = short0 + rng.uniform(-.04, .04)
        leg = rng.uniform(.04, .07)
        top = (long_ + .16 + rng.uniform(-.02, .04), short_ + .16 + rng.uniform(-.02, .04))
        t.update(x=t["x"] + dx, y=t["y"] + dy, yaw=t["yaw"] + dyaw, sx=top[0], sy=top[1],
                 legs=[long_, short_], leg_size=leg)
        c, s_ = math.cos(t["yaw"]), math.sin(t["yaw"])
        for el in link:
            name = el.get("name", "")
            if not name.startswith(t["name"] + "_"):
                continue
            role = name[len(t["name"]) + 1:].replace("_visual", "")
            pose = el.find("pose")
            size = el.find("geometry/box/size")
            if role == "top":
                pose.text = f"{t['x']} {t['y']} 0.725 0 0 {t['yaw']}"
                size.text = f"{top[0]} {top[1]} 0.05"
            elif role.startswith("leg_"):
                i, j = (int(v) for v in role.split("_")[1:3])
                lx, ly = i * long_ / 2, j * short_ / 2
                pose.text = f"{t['x'] + c * lx - s_ * ly} {t['y'] + s_ * lx + c * ly} 0.35 0 0 {t['yaw']}"
                size.text = f"{leg} {leg} 0.7"
        new_layout.append(t)
    return new_layout


def sample_wall_bumps(world, walls, rng):
    """Uneven long walls: pillars, radiators or cabinets standing against the wall face
    (one or two per long wall). They are named wall_* so prints still need a flat face."""
    link = world.find("model[@name='slam_room']/link")
    bumps = []
    for top in (True, False):
        want = int(rng.integers(1, 3))  # one or two per long wall
        placed_here = 0
        for _ in range(40):
            if placed_here >= want:
                break
            x = float(rng.uniform(-8.3, 6.8))
            width = float(rng.uniform(.2, 1.2))
            faces = [wall_face(walls, x + d, top) for d in np.linspace(-width / 2, width / 2, 5)]
            if None in faces or np.ptp(faces) > .08:
                continue
            face = max(faces) if not top else min(faces)
            depth = float(rng.uniform(.05, .30))
            height = float(rng.uniform(.5, 2.5))
            y = face - depth / 2 if top else face + depth / 2
            if any(abs(x - b_["x"]) < (width + b_["sx"]) / 2 + .3 and (b_["y"] > 0) == top for b_ in bumps):
                continue
            placed_here += 1
            bumps.append(dict(x=x, y=y, sx=width, sy=depth, height=height))
            for tag in ("collision", "visual"):
                el = ET.SubElement(link, tag, name=f"wall_bump_{len(bumps)}_{tag}")
                ET.SubElement(el, "pose").text = f"{x:.4f} {y:.4f} {height / 2:.4f} 0 0 0"
                ET.SubElement(ET.SubElement(ET.SubElement(el, "geometry"), "box"), "size").text = (
                    f"{width + .02:.4f} {depth + .02:.4f} {height:.4f}")
                if tag == "visual":
                    mat = ET.SubElement(el, "material")
                    for k in ("ambient", "diffuse"):
                        ET.SubElement(mat, k).text = ".84 .83 .80 1"
    return bumps


def sample_debris(world, occ, rng, start, targets):
    """Small floor clutter: low pieces under the LiDAR plane (camera and bumper only),
    bottles (a single LiDAR return) and bags or small boxes. Never on the start area, within
    0.8 m of a print, or on another object. Returns the placed items."""
    items = []
    for _ in range(int(rng.integers(3, 9))):
        kind = str(rng.choice(["low", "low", "bottle", "bag"]))
        if kind == "low":
            sx, sy, h = rng.uniform(.05, .12), rng.uniform(.05, .12), rng.uniform(.03, .08)
        elif kind == "bottle":
            sx = sy = rng.uniform(.06, .10); h = rng.uniform(.20, .30)
        else:
            sx, sy, h = rng.uniform(.15, .30), rng.uniform(.12, .25), rng.uniform(.15, .25)
        for _ in range(300):
            x, y = float(rng.uniform(-7.8, 6.8)), float(rng.uniform(-5.0, 5.0))
            body = dict(x=x, y=y, yaw=float(rng.uniform(-math.pi, math.pi)), sx=float(sx), sy=float(sy))
            trial = np.zeros(SHAPE, bool)
            fill(trial, corners(body, .05))
            if (trial & occ).any() or math.dist((x, y), start) < 1.0:
                continue
            if any(math.dist((x, y), (t["x"], t["y"])) < .8 for t in targets):
                continue
            if any(math.dist((x, y), (i["x"], i["y"])) < .4 for i in items):
                continue
            items.append(dict(body, kind=kind, height=float(h)))
            fill(occ, corners(body))
            break
    for k, it in enumerate(items):
        m = ET.SubElement(world, "model", name=f"debris_{k}")
        ET.SubElement(m, "static").text = "true"
        ET.SubElement(m, "pose").text = f"{it['x']} {it['y']} {it['height'] / 2} 0 0 {it['yaw']}"
        lk = ET.SubElement(m, "link", name="body")
        for tag in ("collision", "visual"):
            el = ET.SubElement(lk, tag, name=tag)
            geo = ET.SubElement(el, "geometry")
            if it["kind"] == "bottle":
                cyl = ET.SubElement(geo, "cylinder")
                ET.SubElement(cyl, "radius").text = str(it["sx"] / 2)
                ET.SubElement(cyl, "length").text = str(it["height"])
            else:
                ET.SubElement(ET.SubElement(geo, "box"), "size").text = f"{it['sx']} {it['sy']} {it['height']}"
            if tag == "visual":
                mat = ET.SubElement(el, "material")
                colour = {"low": ".15 .15 .17 1", "bottle": ".55 .75 .85 1", "bag": ".30 .32 .40 1"}[it["kind"]]
                for kk in ("ambient", "diffuse"):
                    ET.SubElement(mat, kk).text = colour
    return items


def static_model(world, name, x, y, yaw, size, colour, cylinder=False):
    sx, sy, h = size
    m = ET.SubElement(world, "model", name=name)
    ET.SubElement(m, "static").text = "true"
    ET.SubElement(m, "pose").text = f"{x} {y} {h / 2} 0 0 {yaw}"
    lk = ET.SubElement(m, "link", name="body")
    for tag in ("collision", "visual"):
        el = ET.SubElement(lk, tag, name=tag)
        geo = ET.SubElement(el, "geometry")
        if cylinder:
            c = ET.SubElement(geo, "cylinder")
            ET.SubElement(c, "radius").text = str(sx / 2)
            ET.SubElement(c, "length").text = str(h)
        else:
            ET.SubElement(ET.SubElement(geo, "box"), "size").text = f"{sx} {sy} {h}"
        if tag == "visual":
            mat = ET.SubElement(el, "material")
            for k in ("ambient", "diffuse"):
                ET.SubElement(mat, k).text = colour


def sample_cables(world, layout, rng, add=True):
    """Round power cables (1 cm across) lying on the floor under desks: below the LiDAR
    plane, visible to the camera, low enough to drive over."""
    cables = []
    for t in layout:
        if rng.random() > .5:
            continue
        for _ in range(int(rng.integers(1, 3))):
            c, s_ = math.cos(t["yaw"]), math.sin(t["yaw"])
            lx = float(rng.uniform(-t["sx"] / 2, t["sx"] / 2))
            ly = float(rng.uniform(-t["sy"] / 2, t["sy"] / 2))
            x, y = t["x"] + c * lx - s_ * ly, t["y"] + s_ * lx + c * ly
            length = float(rng.uniform(.8, 2.5))
            yaw = float(rng.uniform(-math.pi, math.pi))
            cables.append(dict(x=x, y=y, yaw=yaw, length=length))
            if not add:
                continue  # draws kept so the rest of the layout matches the cabled seed
            m = ET.SubElement(world, "model", name=f"cable_{len(cables)}")
            ET.SubElement(m, "static").text = "true"
            ET.SubElement(m, "pose").text = f"{x} {y} .005 0 {math.pi / 2} {yaw}"
            lk = ET.SubElement(m, "link", name="body")
            for tag in ("collision", "visual"):
                el = ET.SubElement(lk, tag, name=tag)
                cyl = ET.SubElement(ET.SubElement(el, "geometry"), "cylinder")
                ET.SubElement(cyl, "radius").text = ".005"
                ET.SubElement(cyl, "length").text = str(length)
                if tag == "visual":
                    mat = ET.SubElement(el, "material")
                    for k in ("ambient", "diffuse"):
                        ET.SubElement(mat, k).text = ".08 .08 .09 1"
    return cables


def sample_wall_clutter(world, walls, rng, start):
    """Two to four bins or bags standing near the long walls (LiDAR-visible)."""
    items = []
    for _ in range(int(rng.integers(2, 5))):
        for _ in range(200):
            top = bool(rng.integers(2))
            x = float(rng.uniform(-7.5, 6.5))
            face = wall_face(walls, x, top)
            if face is None:
                continue
            size = float(rng.uniform(.3, .6))
            gap = float(rng.uniform(.10, .40))
            y = face - (gap + size / 2) if top else face + gap + size / 2
            if math.dist((x, y), start) < 1.4 or any(math.dist((x, y), (i["x"], i["y"])) < 1.0 for i in items):
                continue
            kind = str(rng.choice(["bin", "bag", "box"]))
            h = float(rng.uniform(.4, .9)) if kind != "bag" else float(rng.uniform(.3, .5))
            items.append(dict(x=x, y=y, size=size, height=h, kind=kind))
            static_model(world, f"wallobj_{len(items)}", x, y, float(rng.uniform(-math.pi, math.pi)),
                         (size, size * (.6 if kind == "bag" else 1.), h),
                         {"bin": ".35 .40 .45 1", "bag": ".20 .22 .30 1", "box": ".55 .45 .32 1"}[kind],
                         cylinder=(kind == "bin"))
            break
    return items


def build(base, dest, seed, count_range, clutter=False, desk_jitter=True, debris=True, bumps=True, demo=False, random_start=False, follower=False, cabled=False):
    rng = np.random.default_rng(seed)
    tree = ET.parse(base / "world/osp_studio.sdf")
    world = tree.find("world")
    ref = json.loads((base / "reference.json").read_text())
    start = ref["start"][:2]
    layout = json.loads((base / "run/simulation/demo_layout.json").read_text())["tables"]
    if desk_jitter:
        layout = perturb_desks(world, layout, json.loads((base / "fit.json").read_text()), rng)
    # Drop the reference day's interior returns and its localized print.
    link = world.find("model[@name='slam_room']/link")
    dropped = 0
    for el in list(link):
        if el.get("name", "").startswith("object_"):
            y = float(el.findtext("pose").split()[1])
            if -4.9 < y < 4.9:
                link.remove(el)
                dropped += 1
    for m in list(world.findall("model")):
        if m.get("name", "").startswith(("target_", "movable_")):
            world.remove(m)
    walls = static_occupancy(world, layout, prefix="wall_")
    wall_bumps = sample_wall_bumps(world, walls, rng) if bumps else []
    if wall_bumps:
        walls = static_occupancy(world, layout, prefix="wall_")
    chair_params, chairs = sample_chairs(world, walls, rng)
    cables, wall_clutter = [], []
    if demo:
        # Demo room: desks, glass-end chairs and the task boxes only, plus cables under the
        # desks and a few objects along the long walls.
        cables = sample_cables(world, layout, rng, add=cabled)
        if not cabled:
            cables = []
        wall_clutter = sample_wall_clutter(world, walls, rng, start)
    chair_mask = np.zeros(SHAPE, bool)
    for x, y, yaw in chairs:
        fill(chair_mask, corners(dict(x=x, y=y, yaw=yaw, sx=.44, sy=.44)))
    global _CHAIR_CLEARANCE
    _CHAIR_CLEARANCE = ndimage.distance_transform_edt(~chair_mask) * RES
    if clutter:
        # Clearance for prints/boxes stays relative to the glass-end clusters only; the
        # extra chairs are obstacles, and the print front-clearance rule keeps prints visible.
        chair_params["clutter"] = sample_clutter_chairs(world, walls, layout, rng, start)
    occ0 = static_occupancy(world, layout)
    start_yaw = float(ref["start"][2])
    if random_start:
        # Any free, reachable spot at least 0.6 m from furniture and walls and 1 m from the
        # glass-end chairs, facing any direction (the middle of the room included).
        clear = ndimage.distance_transform_edt(~occ0) * RES
        ok = reachable(occ0, start) & (clear >= .6) & (_CHAIR_CLEARANCE >= 1.0)
        cells_ok = np.argwhere(ok)
        i, j = cells_ok[int(rng.integers(len(cells_ok)))]
        start = [float(LOW[0] + (i + .5) * RES), float(LOW[1] + (j + .5) * RES)]
        start_yaw = float(rng.uniform(-math.pi, math.pi))
    base_reach = reachable(occ0, start).sum()
    near_start = np.zeros(SHAPE, bool)
    fill(near_start, corners(dict(x=start[0], y=start[1], yaw=0, sx=1.6, sy=1.6)))

    for attempt in range(200):
        occ = occ0.copy()
        bodies = []
        count = int(rng.integers(*count_range))
        ok = True
        # Boxes only. Placement classes: next to a desk (legs within 5-30 cm, where the
        # physical robot struggled), against a long wall, or in open floor. At least one
        # box per layout is next to a desk.
        modes = (["between", "desk"] + [str(m) for m in rng.choice(["between", "desk", "wall", "open"],
                                                                    size=max(0, count - 2))])[:count]
        for i in range(count):
            frame = dict(name=f"movable_{i + 1}", object_id=f"movable_{i + 1}", shape="box",
                         sx=float(rng.uniform(.46, 1.25)), sy=float(rng.uniform(.46, 1.1)),
                         height=float(rng.uniform(.55, .95)), placement=modes[i])
            for _ in range(600):
                cand, margin = place_box(frame, modes[i], rng, layout, walls)
                if cand is None:
                    continue
                parts = [cand]
                trial = np.zeros(SHAPE, bool)
                for p in parts:
                    fill(trial, corners(p, margin))
                if (not (trial & (occ | near_start)).any()
                        and all(clear_of_chairs(corners(q)) for q in parts)):
                    break
            else:
                ok = False
                break
            for p in parts:
                fill(occ, corners(p))
            bodies.extend(parts)
        if not ok:
            continue
        reach = reachable(occ, start)
        # Movable bodies may narrow passages but must not seal off part of the room.
        if reach.sum() < 0.85 * base_reach:
            continue
        targets = []
        ids = rng.choice(1000, size=3, replace=False)
        # Exam rule: two prints on boxes (different boxes when possible), one on an outer wall.
        # One print always on a box wedged between desks; the other on another box.
        between = [k for k, b in enumerate(bodies) if b.get("placement") == "between"]
        others = [k for k in rng.permutation(len(bodies)) if k not in between[:1]]
        hosts = (between[:1] + others)[:2] if bodies else []
        for n, ident in enumerate(ids):
            kind = "object" if n < 2 and bodies else "wall"
            for _ in range(800):
                if kind == "wall":
                    top = bool(rng.integers(2))
                    x = float(rng.uniform(-9.2, 8.9))
                    faces = [wall_face(walls, x + d, top) for d in (-PRINT_SIZE * .65, 0, PRINT_SIZE * .65)]
                    if None in faces or np.ptp(faces) > .03:
                        continue
                    inward = np.array([0., 1. if top else -1.])  # into the structure
                    point = np.array([x, faces[1]]) - .003 * inward
                    host, height, object_id = ("north_wall" if top else "south_wall"), None, "wall"
                else:
                    body = bodies[hosts[n] if n < len(hosts) else rng.integers(len(bodies))]
                    side = int(rng.integers(4))
                    yaw = body["yaw"] + side * math.pi / 2
                    inward = np.array([math.cos(yaw), math.sin(yaw)])
                    tangent = np.array([-inward[1], inward[0]])
                    depth, width = (body["sx"], body["sy"]) if side % 2 == 0 else (body["sy"], body["sx"])
                    allowance = width / 2 - PRINT_SIZE * .65 - .02
                    if allowance < 0:
                        continue
                    point = (np.array([body["x"], body["y"]]) - inward * (depth / 2 + .003)
                             + tangent * rng.uniform(-allowance, allowance))
                    host, height, object_id = body["name"], body["height"], body["object_id"]
                low = max(.25, PRINT_SIZE * .65 + .02)
                high = min(.40, height - PRINT_SIZE * .65 - .02) if height else .40
                if high < low:
                    continue
                z = float(rng.uniform(low, high))
                yaw = math.atan2(-inward[1], -inward[0])  # print normal faces the room
                if not clear_of_chairs(point):
                    continue
                if any(math.dist(point, (t["x"], t["y"])) < PRINT_SIZE * 1.4 for t in targets):
                    continue
                # The print face needs open floor straight in front of it (0.7 m deep,
                # 0.6 m wide): a print facing a neighbouring box or a desk across a narrow
                # gap can only be seen at a grazing angle.
                normal = -inward
                side_dir = np.array([-normal[1], normal[0]])
                blocked = False
                for dd in np.arange(.08, .78, .05):
                    for ll in (-.3, -.15, 0., .15, .3):
                        q = point + normal * dd + side_dir * ll
                        cq = cell(q)
                        if not (0 <= cq[0] < SHAPE[0] and 0 <= cq[1] < SHAPE[1]) or occ[cq]:
                            blocked = True
                if blocked:
                    continue
                # A start-connected standpoint with a clear view in front of the print.
                view = None
                for d in (1.0, 1.3, 0.8, 1.6):
                    for lateral in (0., .3, -.3):
                        tangent = np.array([inward[1], -inward[0]])
                        s = point - inward * d + tangent * lateral
                        c = cell(s)
                        if (0 <= c[0] < SHAPE[0] and 0 <= c[1] < SHAPE[1] and reach[c]
                                and sight(occ, s, point - inward * .06)):
                            view = s
                            break
                    if view is not None:
                        break
                if view is None:
                    continue
                targets.append(dict(id=int(ident), x=float(point[0]), y=float(point[1]), z=z, yaw=yaw,
                                    size=PRINT_SIZE, attached_to=host, mount_object=object_id))
                break
            else:
                ok = False
                break
        if ok:
            break
    else:
        raise ValueError("No valid random layout for seed %d" % seed)

    debris_items = sample_debris(world, occ, rng, start, targets) if debris else []
    for b in bodies:
        m = ET.SubElement(world, "model", name=b["name"])
        ET.SubElement(m, "static").text = "true"
        ET.SubElement(m, "pose").text = f"{b['x']} {b['y']} {b['height'] / 2} 0 0 {b['yaw']}"
        lk = ET.SubElement(m, "link", name="body")
        for tag in ("collision", "visual"):
            el = ET.SubElement(lk, tag, name=tag)
            ET.SubElement(ET.SubElement(ET.SubElement(el, "geometry"), "box"), "size").text = (
                f"{b['sx']} {b['sy']} {b['height']}")
            if tag == "visual":
                mat = ET.SubElement(el, "material")
                for k in ("ambient", "diffuse"):
                    ET.SubElement(mat, k).text = ".62 .48 .33 1"
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    for t in targets:
        m = ET.SubElement(world, "model", name=f"target_{t['id']}")
        ET.SubElement(m, "static").text = "true"
        ET.SubElement(m, "pose").text = f"{t['x']} {t['y']} {t['z']} 0 0 {t['yaw']}"
        ml = ET.SubElement(m, "link", name="print")
        unit = t["size"] / 7

        def pixel(name, xyz, dims, colour):
            v = ET.SubElement(ml, "visual", name=name)
            ET.SubElement(v, "pose").text = " ".join(map(str, xyz)) + " 0 0 0"
            ET.SubElement(ET.SubElement(ET.SubElement(v, "geometry"), "box"), "size").text = dims
            mat = ET.SubElement(v, "material")
            for k in ("ambient", "diffuse"):
                ET.SubElement(mat, k).text = colour

        pixel("paper", [0, 0, 0], f".002 {t['size'] * 1.3} {t['size'] * 1.3}", "1 1 1 1")
        img = (cv2.aruco.generateImageMarker(dictionary, t["id"], 7)
               if hasattr(cv2.aruco, "generateImageMarker") else cv2.aruco.drawMarker(dictionary, t["id"], 7))
        for r, c in zip(*np.where(img == 0)):
            pixel(f"cell_{r}_{c}", [0.003, (c - 3) * unit, (3 - r) * unit], f".002 {unit} {unit}", "0 0 0 1")

    shutil.copytree(base, dest)
    if follower:
        # A person-sized kinematic cylinder moved by follower.py during the trial.
        m = ET.SubElement(world, "model", name="follower")
        ET.SubElement(m, "pose").text = "0 0 -5 0 0 0"
        lk = ET.SubElement(m, "link", name="body")
        ET.SubElement(lk, "kinematic").text = "true"
        ET.SubElement(lk, "gravity").text = "false"
        for tag in ("collision", "visual"):
            el = ET.SubElement(lk, tag, name=tag)
            cyl = ET.SubElement(ET.SubElement(el, "geometry"), "cylinder")
            ET.SubElement(cyl, "radius").text = "0.22"
            ET.SubElement(cyl, "length").text = "1.7"
            if tag == "visual":
                mat = ET.SubElement(el, "material")
                for k in ("ambient", "diffuse"):
                    ET.SubElement(mat, k).text = ".25 .30 .55 1"
    tree.write(dest / "world/osp_studio.sdf", encoding="utf-8", xml_declaration=True)
    demo = json.loads((base / "run/simulation/demo_layout.json").read_text())
    demo["tables"] = layout
    demo["desk_jitter"] = bool(desk_jitter)
    (dest / "run/simulation/demo_layout.json").write_text(json.dumps(demo))
    scene = json.loads((base / "scene.json").read_text())
    scene.update(targets=targets, obstacles=bodies, seed=int(seed),
                 name=f"실물 SLAM 방 · 무작위 물체 seed {seed}",
                 chairs=chair_params, debris=debris_items, wall_bumps=wall_bumps,
                 cables=cables, wall_clutter=wall_clutter, demo=bool(demo),
                 robot_spawn=dict(x=start[0], y=start[1], yaw=start_yaw),
                 robot_start_expected=dict(x=start[0], y=start[1], yaw=start_yaw),
                 randomization=dict(enabled=True, seed=int(seed), generator="room-random-objects",
                                    obstacle_count=len(bodies), dropped_reference_interior_returns=dropped))
    for p in (dest / "scene.json", dest / "run/simulation/scene.json"):
        p.write_text(json.dumps(scene))
    import yaml
    doc = yaml.safe_load((base / "scene.yaml").read_text())
    doc["targets"] = targets
    doc["start"] = [float(start[0]), float(start[1]), start_yaw]
    (dest / "scene.yaml").write_text(yaml.safe_dump(json.loads(json.dumps(doc))))
    # The reference trace belongs to a different object layout; keep it only for display.
    refdoc = json.loads((base / "reference.json").read_text())
    refdoc["markers"] = {}
    refdoc["start"] = [float(start[0]), float(start[1]), start_yaw]
    (dest / "reference.json").write_text(json.dumps(refdoc))
    print(json.dumps(dict(seed=int(seed), start=[round(start[0], 2), round(start[1], 2), round(math.degrees(start_yaw))], cables=len(cables), wall_clutter=[w["kind"] for w in wall_clutter],
                          wall_bumps=len(wall_bumps), debris=[d["kind"] for d in debris_items], placements=[b.get("placement") for b in bodies], chairs={k: (v if k == "count" else {a: b for a, b in v.items() if a != "edge"})
                                                  for k, v in chair_params.items()}, bodies=len(bodies), shapes=[b["shape"] for b in bodies],
                          targets=[(t["id"], t["attached_to"]) for t in targets], attempt=attempt)))


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("base", type=Path)
    p.add_argument("dest", type=Path)
    p.add_argument("--seed", type=int, required=True)
    p.add_argument("--min-objects", type=int, default=3)
    p.add_argument("--max-objects", type=int, default=3)
    p.add_argument("--fixed-desks", action="store_true", help="keep the fitted desk geometry")
    p.add_argument("--no-debris", action="store_true", help="no small floor clutter")
    p.add_argument("--flat-walls", action="store_true", help="no random wall protrusions")
    p.add_argument("--cables", action="store_true", help="demo room: round cables under the desks")
    p.add_argument("--follower", action="store_true", help="add a person that trails the robot (follower.py)")
    p.add_argument("--random-start", action="store_true", help="random robot start position and heading")
    p.add_argument("--demo", action="store_true",
                   help="demo room: desks, glass-end chairs, task boxes, cables under desks, wall-side objects")
    p.add_argument("--clutter-chairs", action="store_true",
                   help="also line long walls and desks with chairs (overfitting check)")
    a = p.parse_args()
    build(a.base, a.dest, a.seed, (a.min_objects, a.max_objects + 1), clutter=a.clutter_chairs,
          desk_jitter=not a.fixed_desks, debris=not (a.no_debris or a.demo),
          bumps=not (a.flat_walls or a.demo), demo=a.demo, random_start=a.random_start, follower=a.follower, cabled=a.cables)
