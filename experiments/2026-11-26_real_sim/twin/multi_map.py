"""Estimate room alignment from complete archived SLAM maps and verify on spatially withheld regions."""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy.ndimage import distance_transform_edt, map_coordinates
from scipy.optimize import minimize
from build_scene import rotate
from persistent_scene import stream, stamp
from scipy.spatial.transform import Rotation


RES = 0.12
LO = np.array([-18.0, -13.0])
SHAPE = (220, 300)


def points(path):
    with np.load(path) as d:
        y, x = np.where(d["data"] >= 65)
        return np.c_[x + 0.5, y + 0.5] * float(d["resolution"]) + d["origin"]


def fit_map(xy, reference):
    center = np.median(xy, axis=0)
    rel = xy - center
    # Spatial blocks preserve neighbouring cells in one partition. Validation
    # tests agreement with a second independently recorded SLAM map.
    blocks = np.floor(rel / 1.5).astype(int)
    fold = (blocks[:, 0] * 7 + blocks[:, 1] * 11) % 3
    test = rel[fold == 0]
    training = rel[fold != 0]
    if min(len(test), len(training)) < 100:
        return {"accepted": False, "reason": "insufficient map support"}
    target = np.zeros(SHAPE, dtype=np.uint8)
    u, v = np.floor((reference - LO) / RES).astype(int).T
    ok = (u >= 0) & (v >= 0) & (u < SHAPE[1]) & (v < SHAPE[0])
    target[v[ok], u[ok]] = 1
    dist = distance_transform_edt(target == 0) * RES
    field = np.exp(-0.5 * (dist / 0.20) ** 2).astype(np.float32)
    candidates = []
    for deg in range(-180, 180, 5):
        angle = math.radians(deg)
        q = np.floor(rotate(training, angle) / RES).astype(int)
        qlo = q.min(0)
        q -= qlo
        w, h = q.max(0) + 1
        if h >= SHAPE[0] or w >= SHAPE[1]:
            continue
        template = np.zeros((h, w), dtype=np.float32)
        template[q[:, 1], q[:, 0]] = 1
        corr = cv2.matchTemplate(field, template, cv2.TM_CCORR) / template.sum()
        for _ in range(2):
            j, i = np.unravel_index(corr.argmax(), corr.shape)
            pos = LO + (np.array([i, j]) - qlo) * RES
            candidates.append((float(corr[j, i]), [*pos, angle]))
            cv2.circle(corr, (i, j), 11, -1, -1)

    def distances(p, data):
        q = (rotate(data, p[2]) + p[:2] - LO) / RES - 0.5
        return map_coordinates(
            dist, [q[:, 1], q[:, 0]], order=1, mode="constant", cval=3
        )

    def same(a, b):
        return (
            np.linalg.norm(np.subtract(a[:2], b[:2])) < 0.65
            and abs(math.remainder(a[2] - b[2], math.tau)) < 0.15
        )

    found = []
    for _, p in sorted(candidates, key=lambda x: -x[0])[:32]:
        if any(same(p, r["p"]) for r in found):
            continue
        fun = lambda v: np.minimum(distances(v, training[::2]), 0.75).mean()
        best = minimize(
            fun,
            p,
            method="Powell",
            bounds=[
                (p[0] - 0.72, p[0] + 0.72),
                (p[1] - 0.72, p[1] + 0.72),
                (p[2] - 0.10, p[2] + 0.10),
            ],
        )
        p = best.x
        if any(same(p, r["p"]) for r in found):
            continue
        d = distances(p, test)
        aligned = rotate(test, p[2]) + p[:2]
        inside = (np.abs(aligned[:, 0] + 0.9) < 6.8) & (
            np.abs(aligned[:, 1] + 0.4) < 4.1
        )
        found.append(
            dict(
                p=p.tolist(),
                fit_cost=float(best.fun),
                holdout_20cm=float((d <= 0.20).mean()),
                holdout_median_m=float(np.median(d)),
                interior_points=int(inside.sum()),
                interior_20cm=float((d[inside] <= 0.20).mean())
                if inside.any()
                else 0.0,
            )
        )
    if len(found) < 2:
        return {"accepted": False, "reason": "alternative unavailable"}
    found.sort(key=lambda r: r["fit_cost"])
    best = found[0]
    margin = best["holdout_20cm"] - max(r["holdout_20cm"] for r in found[1:])
    # Outer wall symmetry alone is insufficient evidence.
    accepted = (
        best["holdout_20cm"] >= 0.75
        and margin >= 0.10
        and best["interior_points"] >= 80
        and best["interior_20cm"] >= 0.65
    )
    return dict(
        accepted=bool(accepted),
        source_center=center.tolist(),
        best=best,
        holdout_margin=margin,
        competing_hypotheses=len(found),
        withheld_points=len(test),
    )


def transformed_track(run, estimate, duration):
    events = map(json.loads, (run / "artifacts/startup.jsonl").read_text().splitlines())
    start = next(v["wall_epoch_s"] for v in events if v["event"] == "mission_started")
    p = estimate["best"]["p"]
    center = np.array(estimate["source_center"])
    out = []
    for _, m in stream(run, ["/pose"]):
        t = stamp(m.header.stamp) - start
        if (
            t < 0
            or (duration is not None and t > duration)
            or m.header.frame_id != "map"
            or (out and t - out[-1][0] < 0.25)
        ):
            continue
        pp = m.pose.pose
        q = pp.orientation
        xy = rotate(np.array([pp.position.x, pp.position.y]) - center, p[2]) + p[:2]
        yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2] + p[2]
        out.append(
            [
                round(t, 4),
                *np.round(xy, 4).tolist(),
                round(math.remainder(yaw, math.tau), 4),
            ]
        )
    return out


def audit(root, logs):
    fit = json.loads((root / "scene16-parity/fit.json").read_text())
    ref = json.loads((root / "scene16-parity/reference.json").read_text())["run"]
    trace = lambda k: sorted((logs / k / "artifacts/policy_decisions").glob("*.npz"))
    reference = (
        rotate(points(trace(ref)[-1]) - fit["map_origin"], -fit["yaw"]) + fit["offset"]
    )
    result = {}
    overlays = {}
    for run, archive in json.loads((root / "library.json").read_text()).items():
        if run == ref:
            continue
        snapshots = trace(run)
        if snapshots:
            result[run] = fit_map(points(snapshots[-1]), reference)
        else:
            result[run] = dict(accepted=False, reason="no decision map")
        r = result[run]
        if "best" in r:
            overlays[run] = dict(
                accepted=r["accepted"],
                wall_and_room_agreement=r["best"]["holdout_20cm"],
                interior_agreement=r["best"]["interior_20cm"],
                margin=r["holdout_margin"],
                trace=transformed_track(logs / run, r, archive["duration"]),
            )

        print(
            run,
            r["accepted"],
            r.get("best", {}).get("holdout_20cm"),
            r.get("holdout_margin"),
            flush=True,
        )
    (root / "map-hypotheses.json").write_text(
        json.dumps(overlays, separators=(",", ":"))
    )
    (root / "multi-map.json").write_text(json.dumps(result, indent=2))


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("root", type=Path)
    p.add_argument("logs", type=Path)
    a = p.parse_args()
    audit(a.root, a.logs)
