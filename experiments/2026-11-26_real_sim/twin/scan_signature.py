"""Fit persistent raw-scanner rays across sessions; evaluate on untouched recordings."""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from persistent_scene import stamp, stream


def measure(run):
    valid, short, absent, short_depth = [], [], [], []
    last = None
    geom = None
    for _, msg in stream(run, ["/scan"]):
        t = stamp(msg.header.stamp)
        if last is not None and t - last < 2:
            continue
        last = t
        here = [len(msg.ranges), msg.angle_min, msg.angle_increment, msg.range_min]
        if geom is not None and not np.allclose(here, geom):
            raise ValueError("Scan lattice changed during " + run.name)
        geom = here
        r = np.asarray(msg.ranges)
        close = np.isfinite(r) & (r > 0) & (r < msg.range_min)
        valid.append(np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max))
        short.append(close)
        absent.append(np.isposinf(r))
        short_depth.append(np.where(close, r, np.nan))
    if len(valid) < 30:
        raise ValueError("Insufficient scans in " + run.name)
    return (
        geom,
        np.mean(valid, axis=0),
        np.mean(short, axis=0),
        np.mean(absent, axis=0),
        np.asarray(short_depth),
    )


def fit(library, logs, heldout):
    sessions = {k: measure(logs / k) for k in library}
    train = [r for k, r in sessions.items() if k not in heldout]
    if not train or not set(heldout) <= sessions.keys():
        raise ValueError("Missing independent validation")
    if any(not np.allclose(r[0], train[0][0]) for r in sessions.values()):
        raise ValueError("Raw scan lattice differs across sessions")
    # Reproduce only persistent scanner-frame events. A foreground surface in the
    # room can make a single session's missing percentage arbitrarily high.
    short = np.flatnonzero(np.min([r[2] for r in train], axis=0) >= 0.98)
    missing = np.flatnonzero(np.min([r[3] for r in train], axis=0) >= 0.98)
    depths = (
        np.nanmedian(np.concatenate([r[4][:, short] for r in train]), axis=0)
        if len(short)
        else []
    )
    idx = np.r_[short, missing]
    if not len(idx):
        raise ValueError("No repeatable raw-scanner sector")
    return dict(
        geometry=train[0][0],
        short=dict(zip(map(str, short), map(float, depths))),
        missing=missing.tolist(),
        train=[k for k in sessions if k not in heldout],
        validation={
            k: dict(
                rays=len(idx),
                valid_fraction_in_selected=float(sessions[k][1][idx].mean()),
                measured_absent_fraction=float(sessions[k][3][missing].mean())
                if len(missing)
                else None,
                measured_short_fraction=float(sessions[k][2][short].mean())
                if len(short)
                else None,
            )
            for k in heldout
        },
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for key in ("library", "logs", "output"):
        p.add_argument(key, type=Path)
    p.add_argument("--heldout", nargs="+", required=True)
    a = p.parse_args()
    a.output.write_text(
        json.dumps(fit(json.loads(a.library.read_text()), a.logs, a.heldout), indent=2)
    )
