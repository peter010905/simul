"""Per-episode rows for randomized twin campaigns (evaluator side only).

Usage: python3 campaign_report.py <board root> > rows.jsonl
Each row: run, seed, policy, success (all three registered within 1.5 m), time to finish,
per-print outcome with its host class (between/desk/wall/open), bumper events, lap end time,
committed chair walls, and the layout features (desk jitter, wall bumps, debris, clutter).
Episodes that ended from infrastructure failures (no metrics, start not acknowledged) are
reported with ok=false so they can be re-run rather than counted.
"""
import glob
import json
import os
import sys


def resolve(path, root):
    """scene.json may be a symlink to the container path /home/ubuntu/classroom-twin/...;
    on the V100 host that tree lives under the board root."""
    if os.path.exists(path):
        return path
    if os.path.islink(path):
        target = os.readlink(path)
        marker = "/classroom-twin/"
        if marker in target:
            alt = os.path.join(root, target.split(marker, 1)[1])
            if os.path.exists(alt):
                return alt
    return None


def rows(root):
    for r in sorted(glob.glob(os.path.join(root, "trial-track-rand-*"))):
        if not os.path.isdir(r):
            continue
        name = os.path.basename(r)[len("trial-track-rand-"):]
        seed, policy = name.split("-", 1)
        metrics = os.path.join(r, "metrics.json")
        if not os.path.exists(metrics):
            yield dict(run=name, seed=seed, policy=policy, ok=False)
            continue
        scene_path = resolve(os.path.join(r, "scene.json"), root)
        if scene_path is None:
            continue
        m = json.load(open(metrics))
        scene = json.load(open(scene_path))
        place = {b["name"]: b.get("placement", "?") for b in scene.get("obstacles", [])}
        prints = []
        for t in scene["targets"]:
            host = t["attached_to"]
            cls = "wall" if host.endswith("_wall") else place.get(host, "?")
            reg = m["registered_and_within_s"].get(str(t["id"]))
            prints.append(dict(id=t["id"], cls=cls, found=reg is not None, t=reg,
                               closest=m["min_ground_truth_m"].get(str(t["id"]))))
        lap_end, walls = None, 0
        log = os.path.join(r, "native/center_wall.jsonl")
        if os.path.exists(log):
            for line in open(log):
                x = json.loads(line)
                if lap_end is None and x.get("phase") not in (None, "wall") and x.get("kind") != "wall_lap":
                    lap_end = x["elapsed"]
                walls = max(walls, len(x.get("chair_walls") or []))
        yield dict(run=name, seed=seed, policy=policy, ok=m["reason"] != "action_cancel_unconfirmed",
                   reason=m["reason"], success=all(p["found"] for p in prints),
                   time=m["elapsed_sim_s"], found=sum(p["found"] for p in prints), prints=prints,
                   bumps=m["bumper_events"], lap_end=lap_end, chair_walls=walls,
                   features=dict(desk_jitter="desk_jitter" in json.dumps(scene.get("randomization", {})) or seed >= "3300",
                                 bumps=len(scene.get("wall_bumps", [])), debris=len(scene.get("debris", [])),
                                 clutter=bool(scene.get("chairs", {}).get("clutter"))))


if __name__ == "__main__":
    for row in rows(sys.argv[1]):
        print(json.dumps(row))
