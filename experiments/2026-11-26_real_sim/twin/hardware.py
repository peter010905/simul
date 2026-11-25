"""Prepare an independent simulation stage using the hardware branches of this exact archived policy."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

import yaml
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from install import install
from persistent_scene import stream, stamp


def replace(path, original, replacement, count=1):
    s = path.read_text()
    if s.count(original) != count:
        raise ValueError(f"Unexpected source in {path}: {original}")
    path.write_text(s.replace(original, replacement))


def prepare(run, stage, profile, pose_run=None):
    meta = json.loads((run / "run.json").read_text())
    parent = Path(meta["workspace"])
    if (
        hashlib.sha256((run / "policy.json").read_bytes()).hexdigest()
        != meta["policy_sha256"]
    ):
        raise ValueError("Recorded weight digest mismatch")
    pose_run = pose_run or run
    start = next(
        json.loads(s)["wall_epoch_s"]
        for s in (pose_run / "artifacts/startup.jsonl").read_text().splitlines()
        if json.loads(s)["event"] == "mission_started"
    )
    best = None
    for _, m in stream(pose_run, ["/odom"]):
        delta = stamp(m.header.stamp) - start
        if best is None or abs(delta) < best[0]:
            best = (abs(delta), m.pose.pose)
        if delta > 0.25:
            break
    if best is None or best[0] > 0.15:
        raise ValueError("Missing start odometry")
    p = best[1]
    q = p.orientation
    initial = [
        float(p.position.x),
        float(p.position.y),
        0.0,
        0.0,
        0.0,
        float(Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2]),
    ] + [0.0] * 9
    install(parent, stage, profile)
    conf = stage / "frozen-config"
    shutil.copy2(run / "policy.json", conf / "policy.json")
    shutil.copy2(
        run / "config/nav2_hardware_profile.yaml", conf / "nav2_hardware_profile.yaml"
    )
    config = stage / "src/osp_mission/config/sim_ekf.yaml"
    ekf = yaml.safe_load(config.read_text())
    ekf["/**"]["ros__parameters"]["initial_state"] = initial
    config.write_text(yaml.safe_dump(ekf))
    launch = stage / "src/osp_mission/launch/robot.launch.py"
    replace(
        launch,
        "    raw_scan=value('scan_topic')",
        """    import os
    if not sim or os.getenv('ROS_DOMAIN_ID') not in ('174','175') or os.getenv('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Archived hardware parity is available only in an isolated Gazebo run')
    raw_scan=value('scan_topic')""",
    )
    replace(
        launch,
        "filtering=not sim and value('body_scan_filter')",
        "filtering=value('body_scan_filter')",
    )
    if "if not sim and value('session')" in launch.read_text():
        replace(launch, "if not sim and value('session')", "if value('session')")
    # Keep shared parent code as-is; select hardware tuning without changing ROS time.
    cases = launch.read_text().count("if not sim:")
    if cases not in (1, 2):
        raise ValueError("Unsupported robot launch tuning")
    replace(launch, "if not sim:", "if sim:", cases)
    det = stage / "src/osp_mission/osp_mission/detector.py"
    replace(
        det,
        "hardware = not self.p('use_sim_time')",
        "hardware = True  # Hardware candidate gate in this isolated reproduction.",
    )
    (stage / "hardware_parity.json").write_text(
        json.dumps(
            dict(
                run=run.name,
                pose_run=pose_run.name,
                initial_state=initial,
                before_after="Same policy weights and mission logic. Hardware navigation, scan, SLAM update and candidate gates in a simulation-only stage.",
            )
        )
    )
    print(initial)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for k in ("run", "stage", "profile"):
        p.add_argument(k, type=Path)
    p.add_argument("--pose-run", type=Path)
    prepare(**vars(p.parse_args()))
