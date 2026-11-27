"""Export randomized-room layouts and run trajectories to one JSON for a browser viewer.

usage: python3 traj_export.py <board root> <out.json> [seed prefixes, default 35,36]
Layout per seed: rectangles (walls, glass, chair posts, desk legs, boxes), wall-side objects,
targets, start pose. Per run: trajectory sampled every 1 s of sim time, found prints with
their time, stall windows (>=30 s with <0.15 m of motion), bumps, end reason.
Evaluator-side only (uses ground truth).
"""
import csv
import glob
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

from campaign_report import resolve, rows as report_rows
from stall_report import stalls

KINDS = ("wall", "glass", "chair", "table")


def layout(scene_dir):
    world = ET.parse(os.path.join(scene_dir, "world", "osp_studio.sdf")).find("world")
    rects, points = [], []
    for m in world.findall("model"):
        name = m.get("name", "")
        mp = [float(v) for v in (m.findtext("pose") or "0 0 0 0 0 0").split()]
        if name.startswith("wallobj_"):
            points.append([round(mp[0], 2), round(mp[1], 2)])
            continue
        if name.startswith(("cable_", "debris_", "follower")):
            continue
        for el in m.iter("collision"):
            size = el.find("geometry/box/size")
            if size is None:
                continue
            sx, sy, _ = map(float, size.text.split())
            if name.startswith("movable_"):
                rects.append([round(mp[0], 3), round(mp[1], 3), round(sx, 3), round(sy, 3),
                              round(mp[5], 4), "box"])
                continue
            if name == "demo_furniture":
                if sx > 1 or sy > 1:
                    continue
                kind = "table"
            else:
                n = el.get("name") or ""
                kind = next((k for k in KINDS if n.startswith(k)), None)
                if kind is None:
                    continue
            x, y = [float(v) for v in (el.findtext("pose") or "0 0").split()[:2]]
            rects.append([round(mp[0] + x, 3), round(mp[1] + y, 3), round(sx, 3), round(sy, 3), 0, kind])
    scene = json.load(open(os.path.join(scene_dir, "scene.json")))
    start = json.load(open(os.path.join(scene_dir, "reference.json")))["start"]
    targets = [dict(id=t["id"], x=round(t["x"], 3), y=round(t["y"], 3), yaw=round(t["yaw"], 3),
                    host=t["attached_to"]) for t in scene["targets"]]
    return dict(rects=rects, objects=points, targets=targets,
                start=[round(v, 3) for v in start[:3]], follower="follower" in json.dumps(
                    [m.get("name") for m in world.findall("model")]))


def main():
    root, out = sys.argv[1], sys.argv[2]
    prefixes = tuple((sys.argv[3] if len(sys.argv) > 3 else "35,36").split(","))
    report = {r["run"]: r for r in report_rows(root, prefixes)}
    seeds, runs = {}, []
    for d in sorted(glob.glob(os.path.join(root, "trial-track-rand-*"))):
        name = os.path.basename(d)[len("trial-track-rand-"):]
        if not os.path.isdir(d) or not name.startswith(prefixes) or "cabled" in name:
            continue
        r = report.get(name)
        if not r or "found" not in r or not os.path.exists(os.path.join(d, "trajectory.csv")):
            continue
        scene_path = resolve(os.path.join(d, "scene.json"), root)
        seed = r["seed"]
        if seed not in seeds:
            seed_dir = os.path.join(root, "random-room", "seed-" + seed)
            seeds[seed] = layout(seed_dir if os.path.isdir(seed_dir) else os.path.dirname(scene_path))
        rows = [(float(x["elapsed_sim_s"]), float(x["world_x"]), float(x["world_y"]))
                for x in csv.DictReader(open(os.path.join(d, "trajectory.csv")))]
        path, last = [], -math.inf
        for t, x, y in rows:
            if t - last >= 1.0:
                path.append([round(t), round(x, 2), round(y, 2)])
                last = t
        runs.append(dict(
            run=name, seed=seed, policy=r["policy"], success=r["success"], found=r["found"],
            time=round(r["time"]), reason=r["reason"], bumps=r["bumps"],
            lap_end=r["lap_end"] and round(r["lap_end"]),
            prints=[dict(id=p["id"], cls=p["cls"], found=p["found"], t=p["t"] and round(p["t"]),
                         closest=p["closest"] and round(p["closest"], 2)) for p in r["prints"]],
            stalls=[[round(t0), round(dt), round(p[0], 2), round(p[1], 2)]
                    for t0, dt, p in stalls([tuple(q) for q in path])],
            path=path))
    json.dump(dict(seeds=seeds, runs=runs), open(out, "w"), separators=(",", ":"))
    print(len(seeds), "seeds", len(runs), "runs")


if __name__ == "__main__":
    main()
