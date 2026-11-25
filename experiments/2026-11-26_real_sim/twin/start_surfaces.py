"""Recover local collision surfaces corroborated both at departure and much later in the episode."""

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import shutil
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree

from build_scene import rotate
from persistent_scene import Transforms, matrix, stamp, stream


def reconstruct(root, run, destination):
    src = root / "scene16-parity"
    ref = json.loads((src / "reference.json").read_text())
    fit = json.loads((src / "fit.json").read_text())
    t0 = next(
        json.loads(s)["wall_epoch_s"]
        for s in (run / "artifacts/startup.jsonl").read_text().splitlines()
        if json.loads(s)["event"] == "mission_started"
    )
    tree = Transforms()
    for topic, msg in stream(run, ["/tf_static", "/pose"]):
        if topic == "/tf_static":
            tree.add(msg, True)
        else:
            p = msg.pose.pose
            tree.dynamic[("map", "base_link")].append(
                (
                    stamp(msg.header.stamp),
                    matrix(
                        SimpleNamespace(rotation=p.orientation, translation=p.position)
                    ),
                )
            )
    tree.finish()
    evidence = np.load(root / "evidence43/evidence.npz")
    row, col = np.nonzero(evidence["grid"] >= 65)
    metric = (
        rotate(
            np.c_[col + 0.5, row + 0.5] * float(evidence["resolution"]),
            evidence["origin"][2],
        )
        + evidence["origin"][:2]
    )
    world = rotate(metric - fit["map_origin"], -fit["yaw"]) + fit["offset"]
    dist = np.linalg.norm(world - np.array(ref["start"][:2]), axis=1)
    nearby = (dist >= 0.43) & (dist < 3.5)
    metric = metric[nearby]
    world = world[nearby]
    groups = defaultdict(list)
    previous = -100
    for _, msg in stream(run, ["/scan_self_filtered"]):
        t = stamp(msg.header.stamp) - t0
        if t < -8 or 3 < t < 30 or t - previous < (0.2 if t <= 3 else 1):
            continue
        try:
            tf = tree.lookup("map", msg.header.frame_id, t + t0)
        except ValueError:
            continue
        previous = t
        yaw = math.atan2(tf[1, 0], tf[0, 0])
        kind = "early" if t <= 3 else "later"
        # Multiple scans in one quantized pose contribute one vote at most.
        key = (
            kind,
            *(np.floor(tf[:2, 3] / 0.30).astype(int)),
            round(yaw / math.radians(20)),
        )
        ranges = np.asarray(msg.ranges)
        valid = (
            np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        )
        ang = (msg.angle_min + np.arange(len(ranges)) * msg.angle_increment)[valid]
        xy = (np.c_[np.cos(ang), np.sin(ang)] * ranges[valid, None]) @ tf[
            :2, :2
        ].T + tf[:2, 3]
        groups[key].append(xy)
    votes = {k: np.zeros(len(metric), int) for k in ("early", "later")}
    for key, chunks in groups.items():
        cloud = np.concatenate(chunks)
        if len(cloud):
            votes[key[0]] += (
                cKDTree(cloud).query(metric, distance_upper_bound=0.10)[0] <= 0.10
            )
    matched = (votes["early"] >= 2) & (votes["later"] >= 2)
    cells = sorted(set(map(tuple, np.floor(world[matched] / 0.08).astype(int))))
    if not cells:
        raise ValueError("No independently repeated departure surfaces")
    sdf = ET.parse(src / "world/osp_studio.sdf")
    model = ET.SubElement(sdf.find("world"), "model", name="departure_episode_returns")
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="low_returns")
    boxes = []
    for j in sorted({j for i, j in cells}):
        xs = np.array(sorted(i for i, k in cells if k == j))
        for span in np.split(xs, np.flatnonzero(np.diff(xs) > 1) + 1):
            x, y, w = (
                float((span[0] + span[-1] + 1) * 0.04),
                float((j + 0.5) * 0.08),
                float(len(span) * 0.08),
            )
            dx = max(0, abs(x - ref["start"][0]) - w / 2)
            dy = max(0, abs(y - ref["start"][1]) - 0.04)
            if math.hypot(dx, dy) < 0.31:
                continue
            boxes.append(dict(x=x, y=y, sx=w, sy=0.08))
            for tag in ("collision", "visual"):
                el = ET.SubElement(link, tag, name=f"{tag}_{len(boxes)}")
                ET.SubElement(el, "pose").text = f"{x} {y} .15 0 0 0"
                ET.SubElement(
                    ET.SubElement(ET.SubElement(el, "geometry"), "box"), "size"
                ).text = f"{w} .08 .30"
                if tag == "visual":
                    ET.SubElement(
                        ET.SubElement(el, "material"), "diffuse"
                    ).text = ".20 .20 .21 1"
    shutil.copytree(src, destination)
    sdf.write(
        destination / "world/osp_studio.sdf", encoding="utf-8", xml_declaration=True
    )
    existing = json.loads((src / "returns.json").read_text())
    (destination / "returns.json").write_text(json.dumps(existing + boxes))
    summary = dict(
        early_window_s=[-8, 3],
        late_after_s=30,
        validation_window_s=[4, 8],
        matched_cells=int(matched.sum()),
        added_strips=len(boxes),
        minimum_early_views=2,
        minimum_late_views=2,
        link_radius_m=0.10,
    )
    (destination / "departure_support.json").write_text(json.dumps(summary))
    print(summary)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for key in ("root", "run", "destination"):
        p.add_argument(key, type=Path)
    reconstruct(**vars(p.parse_args()))
