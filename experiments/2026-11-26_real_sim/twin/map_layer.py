"""Resample the recorded occupancy grid into the registered simulator room frame."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from build_scene import ANCHORS, rotate


def render(root, anchors=None):
    fit = json.loads((root / "scene16-parity/fit.json").read_text())
    d = np.load(root / "evidence43/evidence.npz")
    grid = d["grid"]
    row, col = np.where(grid >= 0)
    if not len(row):
        raise ValueError("No known occupancy")
    res = float(d["resolution"])
    origin = d["origin"]
    metric = rotate(np.c_[col + 0.5, row + 0.5] * res, origin[2]) + origin[:2]
    xy = rotate(metric - fit["map_origin"], -fit["yaw"]) + fit["offset"]
    low = np.floor(np.minimum(xy.min(axis=0), [-10, -6.2])) - 0.25
    high = np.ceil(np.maximum(xy.max(axis=0), [10, 6.2])) + 0.25
    shape = np.ceil((high - low) / res).astype(int)
    x = low[0] + (np.arange(shape[0]) + 0.5) * res
    y = high[1] - (np.arange(shape[1]) + 0.5) * res
    xx, yy = np.meshgrid(x, y)
    physical = (
        rotate(np.c_[xx.ravel(), yy.ravel()] - fit["offset"], fit["yaw"])
        + fit["map_origin"]
    )
    c, r = np.floor(rotate(physical - origin[:2], -origin[2]) / res).astype(int).T
    values = np.full(xx.size, -1)
    ok = (r >= 0) & (c >= 0) & (r < grid.shape[0]) & (c < grid.shape[1])
    values[ok] = grid[r[ok], c[ok]]
    values = values.reshape(xx.shape)
    pix = np.where(values < 0, 216, np.where(values >= 65, 62, 250)).astype(np.uint8)
    path = root / "registered-slam.png"
    if not cv2.imwrite(str(path), pix):
        raise OSError(path)
    meta = dict(
        x=float(low[0]),
        y=float(-high[1]),
        width=float(shape[0] * res),
        height=float(shape[1] * res),
    )
    (root / "registered-slam.json").write_text(json.dumps(meta))
    if anchors:
        doc = json.loads(anchors.read_text())
        ref = json.loads((root / "scene16-parity/reference.json").read_text())
        if doc["run"] != ref["run"]:
            raise ValueError("Anchor coordinates belong to a different SLAM map")
        real = []
        for point in doc["points"]:
            label = point["leg_label"]
            if int(label[1:]) in ANCHORS:
                p = (
                    rotate(np.array(point["map_xy_m"]) - fit["map_origin"], -fit["yaw"])
                    + fit["offset"]
                )
                real.append(dict(id=label, x=float(p[0]), y=float(p[1])))
        (root / "leg-anchors.json").write_text(json.dumps(real))
    print(meta)


def overview(root, out):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    fit = json.loads((root / "scene16-parity/fit.json").read_text())
    desks = json.loads(
        (root / "scene16-parity/run/simulation/demo_layout.json").read_text()
    )["tables"]
    anchors = json.loads((root / "leg-anchors.json").read_text())
    b = json.loads((root / "registered-slam.json").read_text())
    pixel = plt.imread(root / "registered-slam.png")
    fig, ax = plt.subplots(figsize=(13, 7), constrained_layout=True)
    ax.imshow(
        pixel,
        cmap="gray",
        vmin=0,
        vmax=1,
        origin="upper",
        extent=(b["x"], b["x"] + b["width"], -b["y"] - b["height"], -b["y"]),
        interpolation="nearest",
    )
    corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])
    for d in desks:
        center = [d["x"], d["y"]]
        a = d["yaw"]
        outline = rotate(corners * [d["sx"] / 2, d["sy"] / 2], a) + center
        ax.add_patch(
            Polygon(outline, facecolor="#d6ae6350", edgecolor="#80552d", linewidth=1)
        )
        legs = rotate(corners * np.array(fit["leg_dimensions"]) / 2, a) + center
        ax.scatter(
            *legs.T,
            color="#9c4804",
            marker="s",
            s=15,
            label="64 model legs" if d == desks[0] else None,
        )
    ax.scatter(
        [i["x"] for i in anchors],
        [i["y"] for i in anchors],
        facecolors="none",
        edgecolors="#0071bb",
        linewidths=1.5,
        s=57,
        label="14 SLAM correspondences",
    )
    ax.set(
        aspect="equal",
        xlabel="Room x (m)",
        ylabel="Room y (m)",
        title="16 equal-sized desks on the registered physical SLAM map",
    )
    ax.legend(loc="upper left", framealpha=0.94, ncol=2)
    fig.savefig(out, dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("root", type=Path)
    p.add_argument("--anchors", type=Path)
    p.add_argument("--plot", type=Path)
    args = p.parse_args()
    render(args.root, args.anchors)
    if args.plot:
        overview(args.root, args.plot)
