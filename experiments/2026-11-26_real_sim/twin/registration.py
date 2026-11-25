"""Estimate start SE(2) from startup LiDAR, then check disjoint later scans and alternative poses."""

import argparse
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np
from scipy.ndimage import map_coordinates, distance_transform_edt
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from persistent_scene import Transforms, stream, stamp
from build_scene import rotate

RES = 0.075
ORIGIN = np.array([-17.0, -14.0])


def endpoints(run):
    events = [
        json.loads(l)
        for l in (run / "artifacts/startup.jsonl").read_text().splitlines()
    ]
    t0 = next(e["wall_epoch_s"] for e in events if e["event"] == "mission_started")
    tf = Transforms()
    for _, msg in stream(run, ["/tf_static"]):
        tf.add(msg, True)
    tf.finish()
    od = []
    for _, m in stream(run, ["/odom"]):
        t = stamp(m.header.stamp) - t0
        if t > 25:
            break
        if t < -1:
            continue
        p = m.pose.pose
        q = p.orientation
        od.append(
            [
                t,
                p.position.x,
                p.position.y,
                Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2],
            ]
        )
    od = np.array(od)
    if not len(od) or od[0, 0] > 0 or od[-1, 0] < 19:
        raise ValueError("Missing startup odometry")
    od[:, 3] = np.unwrap(od[:, 3])
    pose = lambda t: np.array([np.interp(t, od[:, 0], od[:, i]) for i in (1, 2, 3)])
    p0 = pose(0)
    out = [[], []]
    last = -10
    for _, m in stream(run, ["/scan_self_filtered"]):
        t = stamp(m.header.stamp) - t0
        if t > 22:
            break
        if t < 0 or t - last < 0.82 or 5 < t < 12:
            continue
        last = t
        p = pose(t)
        ext = tf.lookup("base_link", m.header.frame_id, t + t0)
        rho = np.array(m.ranges)
        a = m.angle_min + np.arange(len(rho)) * m.angle_increment
        valid = (
            np.isfinite(rho)
            & (rho >= max(0.25, m.range_min))
            & (rho <= min(8.0, m.range_max))
        )
        xy = np.c_[rho[valid] * np.cos(a[valid]), rho[valid] * np.sin(a[valid])]
        b = xy @ ext[:2, :2].T + ext[:2, 3]
        out[int(t > 5)].extend(rotate(b, p[2] - p0[2]) + rotate(p[:2] - p0[:2], -p0[2]))
    groups = [np.unique(np.rint(np.array(g) / 0.10), axis=0) * 0.10 for g in out]
    if min(map(len, groups)) < 100:
        raise ValueError("Insufficient scan endpoints")
    return p0, groups


def match(grid, points, test):
    strength = np.exp(-(np.minimum(grid, 1.0) ** 2) / (2 * 0.17**2)).astype("float32")
    best = []
    for yaw in np.deg2rad(np.arange(-180, 180, 5)):
        pix = np.unique(np.rint(rotate(points, yaw) / RES).astype(int), axis=0)
        half = int(np.abs(pix).max()) + 1
        kernel = np.zeros((2 * half + 1, 2 * half + 1), np.float32)
        kernel[pix[:, 1] + half, pix[:, 0] + half] = 1.0 / len(pix)
        scores = cv2.filter2D(strength, -1, kernel, borderType=cv2.BORDER_CONSTANT)
        corners = (np.array([[-9.8, -5.6], [9.8, 5.6]]) - ORIGIN) / RES
        x, y = corners[0].astype(int)
        x1, y1 = corners[1].astype(int)
        part = scores[y:y1, x:x1].copy()
        for _ in range(3):
            j, i = np.unravel_index(np.argmax(part), part.shape)
            best.append(
                (float(part[j, i]), *(ORIGIN + np.array([x + i, y + j]) * RES), yaw)
            )
            cv2.circle(part, (i, j), 12, -1, -1)
    solutions = []

    def distances(params, pts):
        xy = (rotate(pts, params[2]) + params[:2] - ORIGIN) / RES
        return map_coordinates(
            grid, [xy[:, 1], xy[:, 0]], order=1, mode="constant", cval=2.0
        )

    def duplicate(p, q):
        return (
            np.linalg.norm(p[:2] - q[:2]) < 0.7
            and abs(math.remainder(p[2] - q[2], math.tau)) < 0.25
        )

    for _, x, y, a in sorted(best, reverse=True)[:55]:
        init = np.array([x, y, a])
        if any(duplicate(init, q[1]) for q in solutions):
            continue
        fit = minimize(
            lambda p: np.mean(np.minimum(distances(p, points), 0.55)),
            init,
            method="Powell",
            bounds=[(x - 0.45, x + 0.45), (y - 0.45, y + 0.45), (a - 0.13, a + 0.13)],
        )
        if not any(duplicate(fit.x, q[1]) for q in solutions):
            solutions.append((fit.fun, fit.x))
    solutions.sort(key=lambda q: q[0])
    out = []
    for score, p in solutions[:5]:
        ds = distances(p, test)
        out.append(
            dict(
                pose=p.tolist(),
                train_clipped_mean=float(score),
                later_clipped_mean=float(np.mean(np.minimum(ds, 0.55))),
                later_within_20cm=float(np.mean(ds < 0.20)),
            )
        )
    return out


def register(audit, logs, evidence, fixture):
    fit = json.loads((fixture / "fit.json").read_text())
    d = np.load(evidence)
    r, c = np.nonzero(d["persistent"])
    xy = (
        rotate(
            (np.c_[c + 0.5, r + 0.5] * float(d["resolution"]) + d["origin"][:2])
            - fit["map_origin"],
            -fit["yaw"],
        )
        + fit["offset"]
    )
    grid = np.zeros((380, 460), bool)
    pix = np.rint((xy - ORIGIN) / RES).astype(int)
    grid[pix[:, 1], pix[:, 0]] = 1
    dist = distance_transform_edt(~grid) * RES
    result = {}
    for a in json.loads(audit.read_text()):
        run = logs / a["run"]
        try:
            p0, (train, test) = endpoints(run)
            candidates = match(dist, train, test)
            top = candidates[0]
            separation = (
                min((q["later_clipped_mean"] for q in candidates[1:]), default=0)
                - top["later_clipped_mean"]
            )
            accepted = (
                len(candidates) >= 2
                and separation > 0.08
                and top["later_within_20cm"] > 0.70
            )
            result[run.name] = dict(
                odom=p0.tolist(),
                candidates=candidates,
                accepted=bool(accepted),
                test_margin_m=separation,
                train_unique_points=len(train),
                later_unique_points=len(test),
            )
        except (ValueError, IndexError) as e:
            result[run.name] = dict(accepted=False, error=str(e))
        print(
            run.name,
            result[run.name].get("accepted"),
            result[run.name].get("candidates", [None])[0],
            flush=True,
        )
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for k in ("audit", "logs", "evidence", "fixture", "output"):
        p.add_argument(k, type=Path)
    a = p.parse_args()
    a.output.write_text(
        json.dumps(register(a.audit, a.logs, a.evidence, a.fixture), indent=2)
    )
