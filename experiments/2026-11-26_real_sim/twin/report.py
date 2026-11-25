"""Summarize completed native episodes with aligned clocks and explicit coverage."""

import argparse
import json
from pathlib import Path
import numpy as np
from comparison import trial, sample, first_displacements, compare


def sweep(rows):
    a = np.asarray(rows)
    if a.ndim != 2 or a.shape[1] < 4:
        return None
    a = a[(a[:, 0] >= 1) & (a[:, 0] <= 7)]
    if len(a) < 5:
        return None
    return float(np.median(np.diff(np.unwrap(a[:, 3])) / np.diff(a[:, 0])))


def report(root):
    ref = json.loads((root / "scene16-parity/reference.json").read_text())
    lib = json.loads((root / "library.json").read_text())
    phase = []
    for v in lib.values():
        phase.append(sweep(v["traces"]["/odom"]))
    summary = dict(
        real_drive_s=ref["duration"],
        real_stop=ref["stop_reason"],
        real_displacements_s=first_displacements(ref["trace"], ref["start"][:2]),
        real_registered_s=ref["first_seen"],
        real_ids=list(map(int, ref["markers"])),
        real_sweep_rad_s_by_session=phase,
        geometry=json.loads((root / "scene16-parity/fit.json").read_text()),
        other_run_start_registration="No alternative start accepted by withheld scan agreement and ambiguity checks",
        runs=[],
    )
    traces = []
    for folder in sorted(root.glob("trial-track-*")):
        if not folder.is_dir() or not (folder / "metrics.json").is_file():
            continue
        data = trial(folder, ref)
        provenance = folder / "provenance.json"
        proof = json.loads(provenance.read_text()) if provenance.exists() else {}
        if proof.get("policy_run", ref["run"]) != ref["run"]:
            continue
        rows = data.pop("trace")
        if not rows:
            continue
        scene = (folder / "world").resolve().parent.name
        data.update(
            scene=scene,
            sweep=sweep(rows),
            first_60s_position_rmse=trial_window(ref["trace"], rows, 60)
            if rows[-1][0] >= 60
            else None,
            first_8s_heading_median_deg=early_heading(
                lib[ref["run"]]["traces"]["/odom"], rows
            ),
        )
        data["workspace"] = proof.get("workspace", "sim-default")
        data["time_limit_s"] = proof.get("time_limit_s", 600)
        prefix = folder.name.rsplit("-", 1)[-1]
        traces.append(
            (
                f"{prefix} · {scene} · {data['workspace']} ({data['time_limit_s']}s)",
                rows,
            )
        )
        summary["runs"].append(data)
    # Only compare independent repeats of the same scene, stage and time budget.
    repeat_groups = {}
    for item, (_, trace) in zip(summary["runs"], traces):
        path = root / item["id"] / "provenance.json"
        if not path.exists():
            continue
        p = json.loads(path.read_text())
        key = tuple(
            p.get(k)
            for k in (
                "workspace",
                "world_sha256",
                "pose_run",
                "policy_sha256",
                "source_settings_sha256",
            )
        ) + (p.get("time_limit_s", 600),)
        repeat_groups.setdefault(key, []).append((item["id"], trace))
    summary["same_setup_repeats"] = []
    for group in repeat_groups.values():
        for (id1, t1), (id2, t2) in zip(group, group[1:]):
            summary["same_setup_repeats"].append(
                dict(left=id1, right=id2, comparison=compare(t1, t2))
            )
    return ref, summary, traces


def trial_window(real, sim, horizon):
    t = np.arange(0.5, horizon + 0.01, 0.5)
    x = sample(real, t)
    y = sample(sim, t)
    return float(np.sqrt(np.nanmean(np.sum((x[:, :2] - y[:, :2]) ** 2, axis=1))))


def early_heading(real, sim):
    t = np.arange(0.5, 8.01, 0.5)
    dy = sample(real, t)[:, 2] - sample(sim, t)[:, 2]
    return float(np.nanmedian(np.abs(np.degrees(np.arctan2(np.sin(dy), np.cos(dy))))))


def plot(root, ref, traces, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    desks = json.loads(
        (root / "scene16-parity/run/simulation/demo_layout.json").read_text()
    )["tables"]
    fig, axes = plt.subplots(
        len(traces),
        1,
        figsize=(10, 4.5 * len(traces)),
        squeeze=False,
        sharex=True,
        sharey=True,
        constrained_layout=True,
    )
    for ax, (scene, rows) in zip(axes.ravel(), traces):
        for d in desks:
            a = d["yaw"]
            rot = np.array([[np.cos(a), np.sin(a)], [-np.sin(a), np.cos(a)]])
            corners = (
                np.array([[-1, -1], [-1, 1], [1, 1], [1, -1]])
                * np.array([d["sx"], d["sy"]])
                * 0.5
            )
            ax.add_patch(
                Polygon(
                    corners @ rot + [d["x"], d["y"]],
                    fc="#d9cbb5",
                    ec="#ae9675",
                    linewidth=0.6,
                )
            )
        for trace, color, title in [
            (ref["trace"], "#1766ac", "Physical SLAM"),
            (rows, "#c64a1b", "Gazebo pose"),
        ]:
            arr = np.array(trace)
            # Show long camera/pose outages as breaks.
            t = np.arange(0, arr[-1, 0], 0.5)
            pts = sample(arr, t)
            ax.plot(pts[:, 0], pts[:, 1], lw=1.4, color=color, label=title)
        ax.scatter(
            *ref["start"][:2],
            marker="o",
            s=42,
            edgecolors="black",
            facecolors="white",
            zorder=4,
        )
        ax.set(
            xlabel="x (m)",
            ylabel="y (m)",
            aspect="equal",
            title=scene.replace("scene16-departure", "Departure surfaces").replace(
                "twin43_scanner_ws", "calibrated RGB + LiDAR"
            ),
        )
        ax.legend(loc="upper right", ncol=2)
    fig.savefig(path, dpi=140)
    plt.close(fig)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("root", type=Path)
    p.add_argument("--plot", type=Path)
    p.add_argument("--plot-runs", nargs="+", help="Select runs for the figure only")
    p.add_argument(
        "--aggregate",
        type=Path,
        help="Write metrics without maps, positions or imagery",
    )
    a = p.parse_args()
    ref, out, traces = report(a.root)
    if a.plot and traces:
        selected = [
            t
            for r, t in zip(out["runs"], traces)
            if not a.plot_runs or r["id"] in a.plot_runs
        ]
        if not selected:
            p.error("No completed run matches --plot-runs")
        plot(a.root, ref, selected, a.plot)
    if a.aggregate:
        compact = dict(out)
        compact["geometry"] = {
            k: v
            for k, v in out["geometry"].items()
            if k
            in (
                "table_count",
                "leg_dimensions",
                "anchor_count",
                "anchor_rmse_m",
                "assumed_overhang_m",
            )
        }
        compact["reference_sessions_in_common_frame"] = 1
        compact["full_mission_completion_rate_comparable"] = False
        compact["source_commit_available"] = False
        real_goal = json.loads((a.root / "reference_decision.json").read_text())
        compact["real_first_decision"] = {
            k: v for k, v in real_goal.items() if k != "goal"
        }
        compact["runs"] = []
        for row in out["runs"]:
            copy = dict(row)
            decision = row["first_decision"]
            if decision:
                copy["first_decision"] = {
                    k: v for k, v in decision.items() if k != "goal"
                }
                if (
                    row["workspace"] != "sim-default"
                    and decision["goal"]
                    and real_goal["goal"]
                ):
                    copy["first_decision"]["goal_separation_m"] = float(
                        np.linalg.norm(np.subtract(decision["goal"], real_goal["goal"]))
                    )
            compact["runs"].append(copy)
        for filename, key in [
            ("laser-before.json", "sdf_rays_before"),
            ("laser-departure.json", "sdf_rays_after"),
            ("rendered-focal.json", "image_motion"),
            ("signature.json", "raw_lidar_signature"),
        ]:
            path = a.root / filename
            if path.is_file():
                data = json.loads(path.read_text())
                compact[key] = (
                    data["validation"] if filename == "signature.json" else data
                )
        a.aggregate.write_text(json.dumps(compact, indent=2, allow_nan=False) + "\n")
    print(json.dumps(out, indent=2, allow_nan=False))
