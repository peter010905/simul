"""Compare by elapsed mission time, with no fit to the driven paths or time warping."""

import csv
import json
import numpy as np


def sample(rows, times, max_gap=2.0):
    rows = np.asarray(rows, dtype=float)
    if len(rows) < 2:
        return np.full((len(times), 3), np.nan)
    rows = rows[np.all(np.isfinite(rows), axis=1)]
    if len(rows) < 2:
        return np.full((len(times), 3), np.nan)
    rows = rows[np.argsort(rows[:, 0], kind="stable")]
    rows = rows[np.r_[True, np.diff(rows[:, 0]) > 0]]
    if len(rows) < 2:
        return np.full((len(times), 3), np.nan)
    t = rows[:, 0]
    idx = np.minimum(np.searchsorted(t, times), len(t) - 1)
    start = np.maximum(0, idx - 1)
    ok = (
        (times >= t[0])
        & (times <= t[-1])
        & (
            (t[idx] - t[start] <= max_gap)
            | np.isclose(t[idx], times, atol=0.00001, rtol=0)
        )
    )
    values = np.c_[
        np.interp(times, t, rows[:, 1]),
        np.interp(times, t, rows[:, 2]),
        np.interp(times, t, np.unwrap(rows[:, 3])),
    ]
    values[~ok] = np.nan
    return values


def compare(real, sim):
    if len(real) < 2 or len(sim) < 2:
        return {}
    horizon = min(real[-1][0], sim[-1][0])
    t = np.arange(0.5, horizon + 0.00001, 0.5)
    a, b = sample(real, t), sample(sim, t)
    ok = np.isfinite(a[:, 0]) & np.isfinite(b[:, 0])
    if not ok.any():
        return {}
    delta = np.linalg.norm(a[:, :2] - b[:, :2], axis=1)
    yaw = a[:, 2] - b[:, 2]
    yaw = np.abs(np.rad2deg(np.arctan2(np.sin(yaw), np.cos(yaw))))
    valid_steps = ok[:-1] & ok[1:]
    length = lambda v: float(
        np.linalg.norm(np.diff(v[:, :2], axis=0), axis=1)[valid_steps].sum()
    )
    early = (
        np.convolve(
            np.where(ok, delta > 0.5, False).astype(int), np.ones(6, dtype=int), "valid"
        )
        if len(t) >= 6
        else []
    )
    first = np.flatnonzero(np.array(early) == 6)
    return dict(
        rmse=float(np.sqrt(np.mean(delta[ok] ** 2))),
        p95=float(np.percentile(delta[ok], 95)),
        median_heading_error_deg=float(np.median(yaw[ok])),
        within_025=float(np.mean(delta[ok] <= 0.25)),
        real_length=length(a),
        sim_length=length(b),
        coverage=float(np.mean(ok)),
        common_end=float(t[-1]),
        samples=int(ok.sum()),
        first_divergence=float(t[first[0]]) if len(first) else None,
    )


def first_decision(path):
    """Read the first saved planner decision, without fitting to later choices."""
    if not path.is_file():
        return None
    with np.load(path, allow_pickle=False) as z:
        rows = z["candidates"]
        idx = int(z["selected_index"])
        cols = list(z["candidate_columns"])
        goal = (
            [float(rows[idx, cols.index(k)]) for k in ("x", "y")]
            if 0 <= idx < len(rows)
            else None
        )
        return dict(
            elapsed=float(z["elapsed"]),
            known_m2=float(np.sum(z["data"] >= 0) * float(z["resolution"]) ** 2),
            candidates=len(rows),
            goal=goal,
        )


def first_displacements(rows, start_xy):
    """First observed times at distance from start, measured on the same 0.5 s clock."""
    if len(rows) < 2:
        return {}
    times = np.arange(0.5, rows[-1][0] + 0.00001, 0.5)
    positions = sample(rows, times)
    radii = np.linalg.norm(positions[:, :2] - start_xy, axis=1)
    result = {}
    for meters in (1, 3, 5):
        reached = np.flatnonzero(radii >= meters)
        result[str(meters)] = float(times[reached[0]]) if len(reached) else None
    return result


def initial_goal_sequence(artifacts, snapshot, maximum=6):
    """Tie the goal journal's local clock to the independently saved first decision."""
    journal = artifacts / "hybrid_goals.jsonl"
    if not snapshot or not snapshot["goal"] or not journal.is_file():
        return []
    entries = []
    with journal.open() as f:
        for line in f:
            try:
                row = json.loads(line)
                point = [float(row["x"]), float(row["y"])]
                t = float(row["t"])
                kind = str(row["kind"])
            except (KeyError, ValueError, TypeError):
                continue
            if not entries:
                if np.linalg.norm(np.subtract(point, snapshot["goal"])) > 0.03:
                    return []
                origin = t - snapshot["elapsed"]
            entries.append(dict(elapsed=t - origin, kind=kind))
            if len(entries) >= maximum:
                break
    return entries


def headings(path):
    try:
        with path.open() as f:
            reader = csv.DictReader(f)
            rows = []
            for r in reader:
                try:
                    row = [float(r[k]) for k in ("elapsed_sim_s", "x", "y", "yaw_rad")]
                except (ValueError, TypeError, KeyError):
                    continue
                if row[0] >= 0 and (not rows or row[0] - rows[-1][0] >= 0.23):
                    rows.append(row)
            return rows
    except FileNotFoundError:
        return []


def trial(folder, ref):
    rows = headings(folder / "headings.csv")
    result = {}
    if (folder / "metrics.json").is_file():
        result = json.loads((folder / "metrics.json").read_text())
    snapshot = first_decision(folder / "native/policy_decisions/0000.npz")
    return dict(
        id=folder.name,
        trace=rows,
        comparison=compare(ref["trace"], rows),
        elapsed=rows[-1][0] if rows else 0,
        finished=bool(result),
        first_decision=snapshot,
        terminal=result.get("reason"),
        wall_elapsed=result.get("elapsed_wall_s"),
        initial_goals=initial_goal_sequence(folder / "native", snapshot),
        first_displacements=first_displacements(rows, ref["start"][:2]),
        registered_s=result.get("registered_s", {}),
        bumps=result.get("bumper_events"),
        ids=result.get("final_ids"),
        duration=result.get("elapsed_sim_s"),
    )
