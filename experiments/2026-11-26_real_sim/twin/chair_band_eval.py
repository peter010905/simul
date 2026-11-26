"""Evaluate chair_band.ChairBand on every recorded physical SLAM decision snapshot.

Reference episode (20261125T174857): the chair boundaries the user drew on the board are
ground truth in the registered room frame (scene16-parity/fit.json). The error is the mean
|x| distance between the virtual wall and the drawn line over the room width.
Other episodes have no accepted registration, so they are checked for self-consistency:
time to first detection, depth of each band from the room end, stability after detection,
and implausible detections. Also compares the existing EndWalls module (policy code).
Usage: python3 chair_band_eval.py <run_logs/robot> <classroom-twin root> <out.json> [policy src]
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from chair_band import ChairBand, ChairBandTracker
from room_scene import WEST_CHAIR_LINE, east_line

REFERENCE = "20261125T174857-2f02f6e167"


def to_room(xy, fit):
    yaw = fit["yaw"]
    c, s = math.cos(-yaw), math.sin(-yaw)
    d = np.asarray(xy) - fit["map_origin"]
    return np.column_stack((d[:, 0] * c - d[:, 1] * s, d[:, 0] * s + d[:, 1] * c)) + fit["offset"]


def gt_error(band, end, fit):
    v = np.linspace(band.v_range[0] + .5, band.v_range[1] - .5, 40)
    xy = end["wall_u"] * band.u_dir + v[:, None] * band.v_dir
    room = to_room(xy, fit)
    # Decide which drawn line this end corresponds to by its side of the room.
    west = room[:, 0].mean() < 0
    line = np.full(len(room), WEST_CHAIR_LINE) if west else east_line(room[:, 1])
    inside = (room[:, 1] > -5.2) & (room[:, 1] < 4.8)
    return ("west" if west else "east"), float(np.mean(np.abs(room[inside, 0] - line[inside])))


def main():
    logs, root, out = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
    policy = Path(sys.argv[4]) if len(sys.argv) > 4 else None
    EndWalls = Grid = None
    if policy:
        sys.path.insert(0, str(policy))
        from osp_mission.end_walls import EndWalls
        from osp_mission._r32_navigation_core import Grid
    fit = json.loads((root / "scene16-parity/fit.json").read_text())
    report = {}
    for run in sorted(logs.glob("20261125T*")):
        snaps = sorted((run / "artifacts/policy_decisions").glob("*.npz"))
        if len(snaps) < 4:
            continue
        rows = []
        tracker = ChairBandTracker()
        commits = []
        for f in snaps:
            a = np.load(f)
            cells = a["data"]
            origin = (*a["origin"][:2], 0.)
            band = ChairBand(cells, float(a["resolution"]), origin)
            row = dict(t=round(float(a["elapsed"]), 1), ends=[])
            for e in band.ends:
                item = dict(sign=e["sign"], depth=round(e["depth_m"], 2), cov=round(e["coverage"], 2),
                            wall_u=round(e["wall_u"], 2))
                if run.name == REFERENCE:
                    item["side"], item["gt_err"] = gt_error(band, e, fit)
                row["ends"].append(item)
            walls = tracker.update(cells, float(a["resolution"]), origin)
            for k, (p, n) in walls.items():
                if k not in [c["slot"] for c in commits]:
                    commits.append(dict(slot=k, t=row["t"]))
            row["committed"] = {str(k): [round(float(v), 3) for v in (*p, *n)] for k, (p, n) in walls.items()}
            if band.axis is not None:
                row["room_len"] = round(band.u_range[1] - band.u_range[0], 2)
                row["room_wid"] = round(band.v_range[1] - band.v_range[0], 2)
            if EndWalls is not None:
                ew = EndWalls(Grid(cells, float(a["resolution"]), origin))
                row["endwalls"] = len(ew.ends)
            rows.append(row)
        summary = dict(snapshots=len(rows), last_t=rows[-1]["t"])
        for sign in (-1, 1):
            hits = [(r["t"], e) for r in rows for e in r["ends"] if e["sign"] == sign]
            if hits:
                depths = [e["depth"] for _, e in hits]
                walls = [e["wall_u"] for _, e in hits]
                summary[str(sign)] = dict(first_t=hits[0][0], detected_fraction=round(len(hits) / len(rows), 2),
                                          depth_median=round(float(np.median(depths)), 2),
                                          wall_u_std=round(float(np.std(walls)), 2),
                                          depth_range=[min(depths), max(depths)])
                if run.name == REFERENCE:
                    errs = [e["gt_err"] for _, e in hits]
                    summary[str(sign)].update(side=hits[-1][1]["side"],
                                              gt_err_median=round(float(np.median(errs)), 2),
                                              gt_err_max=round(float(max(errs)), 2))
        summary["commits"] = commits
        final = rows[-1]["committed"]
        drift = []
        for r in rows:
            for k, w in r["committed"].items():
                p, n = np.array(w[:2]), np.array(w[2:])
                q = np.array(final[k][:2])
                drift.append(abs(float((p - q) @ n)))
        summary["committed_max_drift_m"] = round(max(drift), 2) if drift else None
        if run.name == REFERENCE:
            summary["committed_room_x"] = {k: round(float(to_room(np.array(w[:2])[None], fit)[0, 0]), 2)
                                           for k, w in final.items()}
        if EndWalls is not None:
            summary["endwalls_detected_fraction"] = round(np.mean([r["endwalls"] > 0 for r in rows]), 2)
        report[run.name] = dict(summary=summary, rows=rows)
        print(run.name, json.dumps(summary))
    out.write_text(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
