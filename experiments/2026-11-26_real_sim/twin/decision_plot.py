"""Save paired first-decision maps, at equal metric scale in robot-centred coordinates."""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D
import numpy as np


def draw(real, simulated, output):
    fig, axes = plt.subplots(
        1, 2, figsize=(10, 5), sharex=True, sharey=True, layout="constrained"
    )
    for ax, path, name in zip(
        axes, (real, simulated), ("Physical SLAM", "Gazebo SLAM")
    ):
        with np.load(path, allow_pickle=False) as d:
            grid = d["data"]
            robot = d["robot"]
            xy = d["origin"] - robot
            cols = list(d["candidate_columns"])
            cand = d["candidates"][:, [cols.index("x"), cols.index("y")]] - robot
            idx = int(d["selected_index"])
            res = float(d["resolution"])
            width, height = np.array(grid.shape[::-1]) * res
            ax.set_facecolor("#d7dbdf")
            ax.imshow(
                np.where(grid < 0, 0, np.where(grid >= 65, 2, 1)),
                origin="lower",
                vmin=0,
                vmax=2,
                extent=(xy[0], xy[0] + width, xy[1], xy[1] + height),
                cmap=ListedColormap(["#d7dbdf", "#ffffff", "#293136"]),
                interpolation="nearest",
            )
            ax.scatter(cand[:, 0], cand[:, 1], s=8, alpha=0.7, c="#8861a9")
            ax.scatter(
                0, 0, s=48, marker="o", edgecolors="black", c="#2fa69a", linewidths=0.8
            )
            if 0 <= idx < len(cand):
                ax.scatter(
                    *cand[idx],
                    s=100,
                    marker="*",
                    c="#da571d",
                    edgecolors="black",
                    linewidths=0.6,
                )
            ax.set_title(
                f"{name}   {float(d['elapsed']):.1f} s\n{np.sum(grid >= 0) * res**2:.1f} m² known   ·   {len(cand)} goals"
            )
            ax.set(
                xlim=(-13, 13),
                ylim=(-13, 13),
                aspect="equal",
                xlabel="x relative to robot (m)",
            )
    axes[0].set_ylabel("y relative to robot (m)")
    fig.legend(
        handles=[
            Line2D([], [], linestyle="", marker=m, color=c, label=n)
            for m, c, n in [
                ("o", "#2fa69a", "Robot"),
                ("o", "#8861a9", "Candidates"),
                ("*", "#da571d", "Selected"),
            ]
        ],
        loc="lower center",
        bbox_to_anchor=(0.5, -0.065),
        ncol=3,
    )
    fig.savefig(output, dpi=130, bbox_inches="tight", pad_inches=0.13)
    plt.close(fig)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for k in ("real", "simulated", "output"):
        p.add_argument(k, type=Path)
    draw(**vars(p.parse_args()))
