"""Add repeat-supported interior returns at laser height as uncertain scene proxies."""

import argparse
from pathlib import Path
import json
import math
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree
from scipy.ndimage import label
from build_scene import rotate


def enrich(fixture, evidence, out):
    d = np.load(evidence)
    fit = json.loads((fixture / "fit.json").read_text())
    lab, n = label(d["persistent"], np.ones((3, 3)))
    scene = json.loads((fixture / "scene.json").read_text())
    tree = ET.parse(fixture / "world/osp_studio.sdf")
    world = tree.find("world")
    leg_xy = []
    for leg in world.findall('model[@name="demo_furniture"]/link/collision'):
        if "leg_" in leg.get("name", ""):
            leg_xy.append(list(map(float, leg.findtext("pose").split()[:2])))
    nearest = cKDTree(leg_xy)
    model = ET.SubElement(world, "model", name="repeat_supported_returns")
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="returns")
    boxes = []
    cells = set()
    for i in range(1, n + 1):
        r, c = np.where(lab == i)
        if len(r) < 2:
            continue
        xy = np.c_[c + 0.5, r + 0.5] * float(d["resolution"]) + d["origin"][:2]
        xy = rotate(xy - fit["map_origin"], -fit["yaw"]) + fit["offset"]
        # Ignore structural wall traces already represented by the base shell.
        if len(r) > 240 or np.max(np.ptp(xy, axis=0)) > 3.5:
            continue
        if nearest.query(xy)[0].min() < 0.28 and np.ptp(xy, axis=0).max() < 0.55:
            continue
        for x, y in xy:
            if abs(x) > 9.85 or abs(y) > 5.5 or nearest.query([x, y])[0] < 0.18:
                continue
            cells.add((int(np.floor(x / 0.10)), int(np.floor(y / 0.10))))
    # Consecutive occupied bins only. Do not fill gaps between chair/desk legs.
    for iy in sorted({j for _, j in cells}):
        xs = sorted(i for i, j in cells if j == iy)
        for row in np.split(xs, np.flatnonzero(np.diff(xs) > 1) + 1):
            x = (row[0] + row[-1] + 1) * 0.05
            y = (iy + 0.5) * 0.1
            sx = len(row) * 0.10
            boxes.append(dict(x=x, y=y, sx=sx, sy=0.1))
            for tag in ["collision", "visual"]:
                el = ET.SubElement(link, tag, name=f"{tag}_{len(boxes)}")
                ET.SubElement(el, "pose").text = f"{x} {y} .15 0 0 0"
                ET.SubElement(
                    ET.SubElement(ET.SubElement(el, "geometry"), "box"), "size"
                ).text = f"{sx} .10 .30"
                if tag == "visual":
                    mat = ET.SubElement(el, "material")
                    for k in ["ambient", "diffuse"]:
                        ET.SubElement(mat, k).text = ".20 .20 .21 1"
    spawn = scene["robot_spawn"]
    for box in boxes:
        dx = max(0, abs(box["x"] - spawn["x"]) - box["sx"] / 2)
        dy = max(0, abs(box["y"] - spawn["y"]) - box["sy"] / 2)
        if math.hypot(dx, dy) < 0.24:
            raise ValueError("Repeat returns conflict with start footprint")
    shutil.copytree(fixture, out)
    tree.write(out / "world/osp_studio.sdf", encoding="utf-8", xml_declaration=True)
    (out / "returns.json").write_text(json.dumps(boxes))
    print(len(cells), "occupied 10 cm bins in", len(boxes), "collision strips")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for k in ("fixture", "evidence", "out"):
        p.add_argument(k, type=Path)
    enrich(**vars(p.parse_args()))
