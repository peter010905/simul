"""Estimate horizontal pixel motion per radian from real startup turns (no marker/map labels)."""

import argparse
import json
from pathlib import Path
import sys
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from persistent_scene import stream, stamp
from scipy.spatial.transform import Rotation


def measure(run):
    start = next(
        json.loads(s)["wall_epoch_s"]
        for s in (run / "artifacts/startup.jsonl").read_text().splitlines()
        if json.loads(s)["event"] == "mission_started"
    )
    od = []
    frames = []
    info = None
    for name, msg in stream(
        run,
        [
            "/odom",
            "/oakd/rgb/preview/image_raw/compressed",
            "/oakd/rgb/preview/camera_info",
        ],
    ):
        t = stamp(msg.header.stamp) - start
        if t > 8:
            break
        if name.endswith("camera_info"):
            info = msg
        if t < 0:
            continue
        if name == "/odom":
            q = msg.pose.pose.orientation
            yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2]
            od.append([t, yaw, msg.twist.twist.linear.x])
        if name.endswith("compressed") and 0.7 <= t <= 7.3:
            frames.append(
                (
                    t,
                    cv2.imdecode(
                        np.frombuffer(msg.data, "uint8"), cv2.IMREAD_GRAYSCALE
                    ),
                )
            )
    od = np.asarray(od)
    if info is None or len(od) < 10:
        return {}
    return fit_frames(od, frames, info.k[2], info.k[5])


def fit_frames(od, frames, cx, cy):
    yaw = np.unwrap(od[:, 1])
    focals = []
    matches = 0
    for (t, a), (u, b) in zip(frames, frames[1:]):
        if a is None or b is None or not 0.05 < u - t < 0.16:
            continue
        dy = np.interp(u, od[:, 0], yaw) - np.interp(t, od[:, 0], yaw)
        if not 0.035 < abs(dy) < 0.14 or abs(np.interp(t, od[:, 0], od[:, 2])) > 0.03:
            continue
        corners = cv2.goodFeaturesToTrack(a, 250, 0.015, 4)
        if corners is None:
            continue
        nxt, ok, _ = cv2.calcOpticalFlowPyrLK(a, b, corners, None)
        back, rev, _ = cv2.calcOpticalFlowPyrLK(b, a, nxt, None)
        xy = corners[:, 0]
        dx = nxt[:, 0] - xy
        valid = (
            (ok[:, 0] > 0)
            & (rev[:, 0] > 0)
            & (np.linalg.norm(back[:, 0] - xy, axis=1) < 0.65)
        )
        valid &= (
            (abs(xy[:, 0] - cx) < 65)
            & (abs(dx[:, 1]) < 4)
            & (np.abs(xy[:, 1] - cy) < 90)
        )
        f = dx[:, 0] / np.tan(dy)
        rad = f * f - 4 * (xy[:, 0] - cx) * (nxt[:, 0, 0] - cx)
        valid &= (rad > 0) & (f > 90) & (f < 850)
        if valid.sum() < 8:
            continue
        estimate = (f[valid] + np.sqrt(rad[valid])) / 2
        focals.append(np.median(estimate))
        matches += len(estimate)
    if len(focals) < 5:
        return {}
    return dict(
        pairs=len(focals),
        matches=matches,
        focal_quantiles=np.percentile(focals, [10, 50, 90]).tolist(),
        horizontal_unit="pixels_per_radian_at_image_center",
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for key in ("audit", "logs", "out"):
        p.add_argument(key, type=Path)
    a = p.parse_args()
    result = {
        r["run"]: measure(a.logs / r["run"]) for r in json.loads(a.audit.read_text())
    }
    a.out.write_text(json.dumps(result, indent=2))
    print([(k, v.get("focal_quantiles")) for k, v in result.items()])
