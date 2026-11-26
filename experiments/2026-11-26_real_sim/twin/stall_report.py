"""Where did robots stand still? For each run, list 30 s windows with < 0.15 m of motion and
the nearest scene item (cable, wall-side object, box, follower track not included).

usage: python3 stall_report.py <classroom-twin root> [run-name-glob]
"""
import csv
import glob
import json
import math
import sys
from pathlib import Path


def segment_distance(p, c):
    x, y, yaw, L = c["x"], c["y"], c["yaw"], c["length"]
    ax, ay = x - math.cos(yaw) * L / 2, y - math.sin(yaw) * L / 2
    bx, by = x + math.cos(yaw) * L / 2, y + math.sin(yaw) * L / 2
    t = max(0., min(1., ((p[0] - ax) * (bx - ax) + (p[1] - ay) * (by - ay)) / L ** 2))
    return math.dist(p, (ax + t * (bx - ax), ay + t * (by - ay)))


def nearest(scene, p):
    items = [("cable", segment_distance(p, c)) for c in scene.get("cables", [])]
    items += [(w["kind"], math.dist(p, (w["x"], w["y"]))) for w in scene.get("wall_clutter", [])]
    items += [("box", math.dist(p, (b["x"], b["y"]))) for b in scene.get("obstacles", [])]
    return min(items, key=lambda i: i[1], default=("-", math.inf))


def stalls(rows, window=30., motion=.15):
    out, i = [], 0
    while i < len(rows):
        t0, x0, y0 = rows[i]
        j = i
        while j < len(rows) and rows[j][0] - t0 < window:
            j += 1
        if j >= len(rows):
            break
        seg = rows[i:j + 1]
        if max(math.dist((x0, y0), (x, y)) for _, x, y in seg) < motion:
            k = j
            while k + 1 < len(rows) and math.dist((x0, y0), rows[k + 1][1:]) < motion:
                k += 1
            out.append((t0, rows[k][0] - t0, (x0, y0)))
            i = k + 1
        else:
            i += 1
    return out


def main():
    root = Path(sys.argv[1])
    pattern = sys.argv[2] if len(sys.argv) > 2 else "*"
    for run in sorted(glob.glob(str(root / "**" / pattern / "trajectory.csv"), recursive=True)):
        run = Path(run).parent
        try:
            scene = json.loads((run / "scene.json").read_text())
            rows = [(float(r["elapsed_sim_s"]), float(r["world_x"]), float(r["world_y"]))
                    for r in csv.DictReader(open(run / "trajectory.csv"))]
        except (OSError, ValueError, KeyError):
            continue
        found = stalls(rows)
        if found:
            total = sum(d for _, d, _ in found)
            parts = [f"{t:.0f}s+{d:.0f}s@({p[0]:.1f},{p[1]:.1f}) {nearest(scene, p)[0]} {nearest(scene, p)[1]:.2f}m"
                     for t, d, p in found]
            print(f"{run.name}: stalled {total:.0f}s | " + "; ".join(parts))


if __name__ == "__main__":
    main()
