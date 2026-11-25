"""Export real mission-clock tracks without treating independently initialized maps as one frame."""

import argparse
import json
import shutil
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from persistent_scene import stream, stamp
from build_scene import rotate
from comparison import first_decision, initial_goal_sequence


def collect(audit, logs, fixture):
    reference = json.loads((fixture / "reference.json").read_text())
    fit = json.loads((fixture / "fit.json").read_text())
    library = {}
    for a in json.loads(audit.read_text()):
        root = logs / a["run"]
        events = [
            json.loads(s)
            for s in (root / "artifacts/startup.jsonl").read_text().splitlines()
        ]
        start = next((e for e in events if e["event"] == "mission_started"), None)
        if start is None:
            continue
        end = next(
            (
                e
                for e in events
                if e["event"] in ("safety_stop", "stopped")
                and e["wall_epoch_s"] >= start["wall_epoch_s"]
            ),
            None,
        )
        t0 = start["wall_epoch_s"]
        traces = {"/odom": [], "/pose": []}
        for name, m in stream(root, list(traces)):
            t = stamp(m.header.stamp) - t0
            if end and t > end["wall_epoch_s"] - t0:
                continue
            if t < -5.0:
                continue
            rows = traces[name]
            if rows and t - rows[-1][0] < 0.23:
                continue
            p = m.pose.pose
            q = p.orientation
            yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2]
            rows.append([t, p.position.x, p.position.y, yaw])
        same = root.name == reference["run"]
        out = {}
        for topic, rows in traces.items():
            data = np.array(rows)
            if len(data) < 2:
                continue
            if same and topic == "/pose":
                pos = (
                    rotate(data[:, 1:3] - fit["map_origin"], -fit["yaw"])
                    + fit["offset"]
                )
                angle = data[:, 3] - fit["yaw"]
            else:
                idx = np.abs(data[:, 0]).argmin()
                xy0 = data[idx, 1:3]
                yaw0 = data[idx, 3]
                anchor = reference["start"] if same else [0, 0, 0]
                pos = rotate(data[:, 1:3] - xy0, anchor[2] - yaw0) + anchor[:2]
                angle = data[:, 3] + anchor[2] - yaw0
            out[topic] = np.round(
                np.c_[data[:, 0], pos, np.arctan2(np.sin(angle), np.cos(angle))][
                    data[:, 0] >= 0
                ],
                4,
            ).tolist()
        library[root.name] = dict(
            workspace=Path(a["workspace"]).name,
            slam=a["slam"]["max_laser_range"],
            room_frame=same,
            duration=end["wall_epoch_s"] - t0 if end else None,
            stop=end["event"] if end else None,
            traces=out,
        )
    return library


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for name in ("audit", "logs", "fixture", "output"):
        p.add_argument(name, type=Path)
    a = p.parse_args()
    ref = json.loads((a.fixture / "reference.json").read_text())
    snapshot = a.logs / ref["run"] / "artifacts/policy_decisions/0000.npz"
    (a.output.parent / "reference-goals.json").write_text(
        json.dumps(
            initial_goal_sequence(snapshot.parent.parent, first_decision(snapshot))
        )
    )
    if snapshot.is_file():
        local = a.output.parent / "reference-decision.npz"
        if not local.exists() or local.resolve() != snapshot.resolve():
            shutil.copyfile(snapshot, local)
        (a.output.parent / "reference_decision.json").write_text(
            json.dumps(first_decision(local))
        )
    a.output.write_text(
        json.dumps(collect(a.audit, a.logs, a.fixture), separators=(",", ":"))
    )
