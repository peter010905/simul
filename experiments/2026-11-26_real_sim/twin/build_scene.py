"""Fit one rigid, metric room frame from desk supports, then build an offline Gazebo episode."""

import argparse
import copy
import json
import math
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from persistent_scene import stream, stamp


def rotate(xy, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.asarray(xy) @ np.array([[c, s], [-s, c]])


# Correspondences read from the user's marked-up 12 m map; ambiguous blue dots omitted.
# (column -2..+2, row, corner horizontal sign, vertical sign)
ANCHORS = {
    1: (-2, 1, -1, 1),
    2: (-2, 1, 1, 1),
    3: (-1, 1, -1, 1),
    4: (-1, 1, 1, 1),
    6: (-1, 1, 1, -1),
    5: (0, 1, -1, 1),
    7: (0, 1, 1, 1),
    9: (0, 1, -1, -1),
    11: (1, 1, 1, 1),
    16: (2, 1, -1, 1),
    21: (2, 1, -1, -1),
    22: (2, 1, 1, -1),
    20: (0, -1, -1, 1),
    24: (1, -1, 1, 1),
}


def fit(points, heading):
    def predict(spec, params):
        x, y, angle, pitch, spacing, length, width = params
        col, row, side, end = spec
        return rotate(
            [pitch * col + side * width / 2, row * spacing + end * length / 2], angle
        ) + [
            x,
            y,
        ]

    def residual(params):
        return np.ravel(
            [predict(spec, params) - points[key] for key, spec in ANCHORS.items()]
        )

    # The heading resolves the 180-degree ambiguity in the supplied annotated map.
    x, y = points[5] - rotate([-0.42, 3.6], heading)
    result = least_squares(
        residual,
        [x, y, heading, 2.6, 2.7, 1.8, 0.84],
        bounds=(
            [x - 2, y - 2, heading - 0.15, 2, 2, 1.4, 0.5],
            [x + 2, y + 2, heading + 0.15, 3.2, 3.4, 2.2, 1.1],
        ),
        loss="soft_l1",
        f_scale=0.09,
    )
    return result.x, np.linalg.norm(residual(result.x).reshape(-1, 2), axis=1)


def build(base, records, run, dest):
    dest.mkdir(parents=True, exist_ok=False)
    meta = json.loads((records / "desk-specific/desk-legs-174857.json").read_text())
    if meta["run"] != run.name:
        raise ValueError("Annotated legs belong to another SLAM frame")
    angles = json.loads((records / "desk-all-quads.json").read_text())
    heading = next(r["angle"] for r in angles if r["name"] == "174857")
    points = {int(p["leg_label"][1:]): np.array(p["map_xy_m"]) for p in meta["points"]}
    par, errors = fit(points, heading)
    ox, oy, yaw, pitch, gap, long, short = par
    # Original shell references its wooden floor centre at (-.955,-.4).
    offset = np.array([-0.955, -0.4])

    def world(x, y):
        return rotate([x - ox, y - oy], -yaw) + offset

    events = [
        json.loads(s)
        for s in (run / "artifacts/startup.jsonl").read_text().splitlines()
    ]
    t0 = next(r["wall_epoch_s"] for r in events if r["event"] == "mission_started")
    end = next(r for r in events if r["event"] in ("safety_stop", "stopped"))
    poses = []
    odometry = []
    marks = {}
    first_seen = {}
    sizes = {}
    camera = None
    for topic, msg in stream(
        run,
        [
            "/pose",
            "/odom",
            "/targets",
            "/oakd/rgb/preview/camera_info",
            "/mission/marker_observations",
        ],
    ):
        if topic in ("/pose", "/odom"):
            t = stamp(msg.header.stamp) - t0
            if topic == "/pose" and msg.header.frame_id != "map":
                raise ValueError(msg.header.frame_id)
            p = msg.pose.pose
            q = p.orientation
            a = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2]
            (poses if topic == "/pose" else odometry).append(
                [
                    t,
                    *world(p.position.x, p.position.y),
                    math.remainder(a - yaw, math.tau),
                ]
            )
        elif topic.endswith("camera_info"):
            if camera is None:
                camera = (np.asarray(msg.k).reshape(3, 3), np.asarray(msg.d))
        elif topic.endswith("marker_observations"):
            obs = json.loads(msg.data)
            if camera is None or obs.get("range_source") != "lidar":
                continue
            quad = cv2.undistortPoints(
                np.asarray(obs["corners"], dtype=float).reshape(4, 1, 2), *camera
            ).reshape(4, 2)
            heights = np.abs(np.roll(quad[:, 1], -1) - quad[:, 1])
            height = np.mean(np.sort(heights)[-2:]) * obs["camera_point"][2]
            if 0.04 <= height <= 0.8:
                sizes.setdefault(obs["id"], []).append(float(height))
        else:
            for m in msg.markers:
                if m.ns != "point" or m.header.frame_id != "map":
                    continue
                p = m.pose.position
                marks[m.id] = [*world(p.x, p.y), p.z]
                first_seen.setdefault(m.id, stamp(m.header.stamp) - t0)
    poses = np.array(sorted(poses, key=lambda p: p[0]))
    k = np.searchsorted(poses[:, 0], 0)
    if not 0 < k < len(poses):
        raise ValueError("Start outside pose record")
    prior = poses[k - 1]
    odom = np.array(odometry)
    near = odom[(odom[:, 0] >= prior[0]) & (odom[:, 0] <= 0)]
    if poses[k, 0] - prior[0] > 1:
        if (
            len(near) < 2
            or abs(near[-1, 0]) > 0.15
            or np.max(np.linalg.norm(near[:, 1:3] - near[0, 1:3], axis=1)) > 0.015
            or np.ptp(np.unwrap(near[:, 3])) > 0.025
        ):
            raise ValueError("No stationary odometry support across SLAM gap")
        start = prior[1:].tolist()
    else:
        start = [np.interp(0, poses[:, 0], poses[:, j]) for j in (1, 2)]
        start.append(
            math.remainder(np.interp(0, poses[:, 0], np.unwrap(poses[:, 3])), math.tau)
        )
    duration = end["wall_epoch_s"] - t0
    trace = poses[(poses[:, 0] > 0) & (poses[:, 0] <= duration)]
    reference = dict(
        run=run.name,
        workspace=json.loads((run / "run.json").read_text())["workspace"],
        start=start,
        stop_reason=end["event"],
        duration=duration,
        trace=[[0, *start], *trace.tolist()],
        markers=marks,
        first_seen=first_seen,
    )
    (dest / "reference.json").write_text(json.dumps(reference))
    scene = json.loads((base / "scene.json").read_text())
    tree = ET.parse(base / "world/osp_studio.sdf")
    w = tree.find("world")
    link = w.find("model[@name='demo_furniture']/link")
    templates = {
        el.tag: copy.deepcopy(el)
        for el in link
        if el.get("name", "").startswith("table_middle_1_top")
    }
    legs = {
        el.tag: copy.deepcopy(el)
        for el in link
        if el.get("name", "").startswith("table_middle_1_leg_0_0")
    }
    for el in list(link):
        if el.get("name", "").startswith("table_"):
            link.remove(el)
    tables = []

    def table(name, x, y, angle):
        # Leg centres are fitted; 8 cm top overhang is inherited from the exam fixture.
        sx, sy = long + 0.16, short + 0.16
        tables.append(dict(name=name, x=x, y=y, yaw=angle, sx=sx, sy=sy))
        for role, (dx, dy), z, size, pieces in [
            ("top", (0, 0), 0.725, (sx, sy, 0.05), templates)
        ] + [
            (
                f"leg_{i}_{j}",
                (i * long / 2, j * short / 2),
                0.35,
                (0.055, 0.055, 0.7),
                legs,
            )
            for i in (-1, 1)
            for j in (-1, 1)
        ]:
            px, py = rotate([dx, dy], angle) + [x, y]
            for tag, piece in pieces.items():
                el = copy.deepcopy(piece)
                el.set(
                    "name", name + "_" + role + ("_visual" if tag == "visual" else "")
                )
                el.find("pose").text = f"{px} {py} {z} 0 0 {angle}"
                el.find("geometry/box/size").text = " ".join(map(str, size))
                link.append(el)

    for row, sign in [("north", 1), ("south", -1)]:
        for k in range(5):
            table(
                f"table_{row}_{k + 1}",
                offset[0] + (k - 2) * pitch,
                offset[1] + sign * gap,
                math.pi / 2,
            )
    # Inner aisle estimated from middle row ends L25/L26, with left/right symmetry.
    outside = np.mean([world(*points[k])[0] for k in (25, 26)]) - offset[0] + 0.08
    top = long + 0.16
    aisle = max(0.3, outside - 3 * top)
    for i in range(6):
        k = i % 3
        x = (aisle + (k + 0.5) * top) * (1 if i >= 3 else -1)
        table(f"table_middle_{i + 1}", offset[0] + x, offset[1], 0.0)
    for el in list(w):
        if el.get("name", "").startswith(("target_", "movable_")):
            w.remove(el)
    targets = []
    for ident, (x, y, z) in marks.items():
        # In this episode the only localized ID (42) lies on the south side.
        # Estimate print normal from recorded observation pose, size from detector configuration.
        follow = [
            json.loads(l)
            for l in (run / "artifacts/marker_followups.jsonl").read_text().splitlines()
        ]
        views = [world(*r["observer"]) for r in follow if r["id"] == ident]
        if not views:
            continue
        normal = np.median(views, axis=0) - [x, y]
        a = math.atan2(normal[1], normal[0])
        if not sizes.get(ident):
            continue
        size = float(np.median(sizes[ident]))
        unit = size / 7
        targets.append(
            dict(id=ident, x=x, y=y, z=z, yaw=a, size=size, attached_to="SLAM estimate")
        )
        model = ET.SubElement(w, "model", name=f"target_{ident}")
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = f"{x} {y} {z} 0 0 {a}"
        ml = ET.SubElement(model, "link", name="print")

        def pixel(name, xyz, dimensions, colour):
            v = ET.SubElement(ml, "visual", name=name)
            ET.SubElement(v, "pose").text = " ".join(map(str, xyz)) + " 0 0 0"
            ET.SubElement(
                ET.SubElement(ET.SubElement(v, "geometry"), "box"), "size"
            ).text = dimensions
            mat = ET.SubElement(v, "material")
            for tag in ("ambient", "diffuse"):
                ET.SubElement(mat, tag).text = colour

        pixel("paper", [0, 0, 0], f".002 {size * 1.3} {size * 1.3}", "1 1 1 1")
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        marker = (
            cv2.aruco.generateImageMarker(dictionary, ident, 7)
            if hasattr(cv2.aruco, "generateImageMarker")
            else cv2.aruco.drawMarker(dictionary, ident, 7)
        )
        for r, c in zip(*np.where(marker == 0)):
            pixel(
                f"cell_{r}_{c}",
                [0.003, (c - 3) * unit, (3 - r) * unit],
                f".002 {unit} {unit}",
                "0 0 0 1",
            )
    scene.update(
        targets=targets,
        obstacles=[],
        robot_spawn=dict(zip(("x", "y", "yaw"), start)),
        name="실물 기록 재구성",
    )
    scene["robot_start_expected"] = scene["robot_spawn"]
    scene["furniture"]["table_width"] = short + 0.16
    (dest / "world").mkdir()
    shutil.copytree(base / "run/simulation", dest / "run/simulation")
    for el in tree.iter():
        if el.text and "@TEAM_MAP_ROOT@" in el.text:
            el.text = el.text.replace("@TEAM_MAP_ROOT@", str(dest.resolve()))
    tree.write(dest / "world/osp_studio.sdf", encoding="utf-8", xml_declaration=True)
    layout = dict(tables=tables, chair_count=0, dimensions=[long + 0.16, short + 0.16])
    (dest / "run/simulation/demo_layout.json").write_text(json.dumps(layout))
    for p in (dest / "scene.json", dest / "run/simulation/scene.json"):
        p.write_text(json.dumps(scene))
    (dest / "scene.yaml").write_text(
        yaml.safe_dump(
            json.loads(json.dumps(dict(start=start, targets=targets, duration=600)))
        )
    )
    fit_info = dict(
        map_origin=[ox, oy],
        yaw=yaw,
        offset=offset.tolist(),
        pitch=pitch,
        gap=gap,
        print_size_samples=sizes,
        leg_dimensions=[long, short],
        anchor_count=len(errors),
        anchor_rmse_m=float(np.sqrt(np.mean(errors**2))),
        assumed_overhang_m=0.08,
        middle_half_aisle_m=aisle,
        table_count=len(tables),
    )
    (dest / "fit.json").write_text(json.dumps(fit_info))
    print(
        json.dumps(
            dict(
                fit=fit_info, start=start, localized_ids=list(marks), duration=duration
            )
        )
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for key in ("base", "records", "run", "dest"):
        p.add_argument(key, type=Path)
    build(**vars(p.parse_args()))
