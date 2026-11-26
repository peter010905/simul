"""Evaluate idealized SDF range returns at recorded physical poses, separate from policy error."""

import argparse
import json
import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np

from build_scene import rotate
from persistent_scene import Transforms, stream, stamp, matrix


def local_pose(element):
    p = np.fromstring(element.findtext("pose", "0 0 0 0 0 0"), sep=" ")
    if np.max(np.abs(p[3:5])) > 1e-5:
        raise ValueError("Tilted primitive outside planar scan model")
    return p[:2], p[2], p[5]


def boxes(scene, height):
    out = []
    for model in ET.parse(scene / "world/osp_studio.sdf").findall("./world/model"):
        xy, z, yaw = local_pose(model)
        for link in model.findall("link"):
            lx, lz, la = local_pose(link)
            for col in link.findall("collision"):
                # Collision-only glass is invisible to the GPU LiDAR, as in the room.
                if col.get("name", "").startswith("glass"):
                    continue
                size = col.findtext("geometry/box/size")
                if not size:
                    continue
                w = np.fromstring(size, sep=" ")
                cx, cz, ca = local_pose(col)
                if abs(height - (z + lz + cz)) > w[2] * 0.5:
                    continue
                centre = xy + rotate(lx, yaw) + rotate(cx, yaw + la)
                out.append([*centre, w[0] / 2, w[1] / 2, yaw + la + ca])
    return np.asarray(out)


def predict(origin, angles, rects, limit):
    best = np.full(len(angles), limit)
    for x, y, hw, hh, a in rects:
        off = rotate(origin - [x, y], -a)
        dirs = np.c_[np.cos(angles - a), np.sin(angles - a)]
        dirs = np.where(abs(dirs) < 1e-10, 1e-10, dirs)
        v0 = (-np.array([hw, hh]) - off) / dirs
        v1 = (np.array([hw, hh]) - off) / dirs
        enter = np.maximum(0, np.minimum(v0, v1).max(axis=1))
        leave = np.maximum(v0, v1).min(axis=1)
        best = np.minimum(best, np.where(leave >= enter, enter, limit))
    return best


def stats(rows):
    data = np.concatenate(rows)
    real, sim, limit = data.T
    valid = np.isfinite(real) & (real > 0) & (real <= limit)
    pred = sim < limit
    both = valid & pred
    return dict(
        rays=len(data),
        real_valid=float(valid.mean()),
        model_valid=float(pred.mean()),
        real_p90_m=float(np.percentile(real[valid], 90)),
        model_p90_m=float(np.percentile(sim[pred], 90)),
        common_mae_m=float(np.abs(real[both] - sim[both]).mean()),
        within_20cm_on_common=float(np.mean(np.abs(real[both] - sim[both]) <= 0.20)),
        model_farther_over_50cm=float(np.mean((sim[valid] - real[valid]) > 0.50)),
    )


def repeated_ranges(grid, resolution, origin, origin_yaw, laser_xy, angles, limit):
    # Half-cell steps keep narrow returns in the idealized horizontal slice.
    steps = np.arange(resolution * 0.5, limit, resolution * 0.5)
    laser = rotate(laser_xy - origin, -origin_yaw)
    dirs = angles - origin_yaw
    xs = np.floor((laser[0] + np.cos(dirs)[:, None] * steps) / resolution).astype(int)
    ys = np.floor((laser[1] + np.sin(dirs)[:, None] * steps) / resolution).astype(int)
    inside = (xs >= 0) & (xs < grid.shape[1]) & (ys >= 0) & (ys < grid.shape[0])
    hit = inside & grid[ys.clip(0, grid.shape[0] - 1), xs.clip(0, grid.shape[1] - 1)]
    return np.where(hit.any(axis=1), steps[hit.argmax(axis=1)], limit)


def examine(root, run, evidence=None, scene_name="scene16-parity"):
    scene = root / scene_name
    fit = json.loads((scene / "fit.json").read_text())
    tf = Transforms()
    start = next(
        json.loads(s)["wall_epoch_s"]
        for s in (run / "artifacts/startup.jsonl").read_text().splitlines()
        if json.loads(s)["event"] == "mission_started"
    )
    for topic, m in stream(run, ["/tf_static", "/pose"]):
        if topic == "/tf_static":
            tf.add(m, True)
        else:
            p = m.pose.pose
            tf.dynamic[("map", "base_link")].append(
                (
                    stamp(m.header.stamp),
                    matrix(
                        SimpleNamespace(rotation=p.orientation, translation=p.position)
                    ),
                )
            )
    tf.finish()
    rec = {"startup": [], "drive": [], "held_startup_4_to_8": []}
    if evidence is not None:
        occupancy = np.load(evidence)
        rec.update({"all_repeated_startup": [], "all_repeated_drive": []})
    prev = -99
    rects = None
    for _, m in stream(run, ["/scan_self_filtered"]):
        ts = stamp(m.header.stamp)
        t = ts - start
        if t < 0 or t - prev < (1.0 if t < 8 else 5.0):
            continue
        try:
            tr = tf.lookup("map", m.header.frame_id, ts)
        except ValueError:
            continue
        prev = t
        if rects is None:
            rects = boxes(scene, tr[2, 3])
        yaw = math.atan2(tr[1, 0], tr[0, 0]) - fit["yaw"]
        xy = rotate(tr[:2, 3] - fit["map_origin"], -fit["yaw"]) + fit["offset"]
        angles = yaw + m.angle_min + np.arange(len(m.ranges)) * m.angle_increment
        rho = np.asarray(m.ranges)
        rho = np.where(rho >= m.range_min, rho, np.inf)
        model = predict(xy, angles, rects, m.range_max)
        split = "startup" if t < 8 else "drive"
        rec[split].append(np.c_[rho, model, np.full(len(rho), m.range_max)])
        if 4 <= t < 8:
            rec["held_startup_4_to_8"].append(rec[split][-1])
        if evidence is not None:
            dirs = angles + fit["yaw"]
            alternative = repeated_ranges(
                occupancy["persistent"],
                float(occupancy["resolution"]),
                occupancy["origin"][:2],
                float(occupancy["origin"][2]),
                tr[:2, 3],
                dirs,
                m.range_max,
            )
            rec["all_repeated_" + split].append(
                np.c_[rho, alternative, np.full(len(rho), m.range_max)]
            )
    return {k: dict(scans=len(v), **stats(v)) for k, v in rec.items() if v}


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for name in ("root", "run", "out"):
        p.add_argument(name, type=Path)
    p.add_argument("--evidence", type=Path)
    p.add_argument("--scene", default="scene16-parity")
    a = p.parse_args()
    a.out.write_text(json.dumps(examine(a.root, a.run, a.evidence, a.scene), indent=2))
