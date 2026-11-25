import json
import math
import os
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import cv2
import numpy as np

from comparison import compare, sample, first_displacements
from build_scene import rotate


class MetricTests(unittest.TestCase):
    def test_fixed_clock(self):
        r = [[t, t, 0, 0] for t in range(11)]
        self.assertEqual(compare(r, r)["rmse"], 0)
        s = [[t, t + 1, 0, math.pi] for t in range(11)]
        out = compare(r, s)
        self.assertAlmostEqual(out["rmse"], 1.0)
        self.assertAlmostEqual(out["first_divergence"], 0.5)
        self.assertAlmostEqual(out["median_heading_error_deg"], 180.0)

    def test_gap(self):
        x = sample([[0, 1, 1, 0], [6, 4, 2, 0]], [0, 2.0, 6])
        self.assertTrue(np.isnan(x[1, 0]))
        self.assertEqual(x[2, 0], 4)

    def test_angles(self):
        p = sample([[0, 0, 0, 3.1], [1, 0, 0, -3.1]], [0.5])
        self.assertAlmostEqual(p[0, 2], math.pi)

    def test_distance_uses_displacement(self):
        rows = [[0, 0, 0, 0], [1, 1, 0, 0], [2, 0, 0, 0], [3, 1, 0, 0], [4, 0, 0, 0]]
        self.assertEqual(
            first_displacements(rows, [0, 0]), {"1": 1.0, "3": None, "5": None}
        )

    def test_rotations(self):
        np.testing.assert_allclose(rotate([1.0, 0.0], np.pi / 2), [0, 1], atol=1e-9)


@unittest.skipUnless(os.environ.get("TWIN_FIXTURE"), "native fixture required")
class SceneTests(unittest.TestCase):
    def setUp(self):
        self.root = Path(os.environ["TWIN_FIXTURE"])
        self.world = ET.parse(self.root / "world/osp_studio.sdf").find("world")

    def test_tables(self):
        tables = json.loads(
            (self.root / "run/simulation/demo_layout.json").read_text()
        )["tables"]
        self.assertEqual(len(tables), 16)
        self.assertEqual(len({(t["sx"], t["sy"]) for t in tables}), 1)
        p = self.world.find("model[@name='demo_furniture']/link")
        self.assertEqual(
            len([e for e in p.findall("collision") if "_leg_" in e.get("name", "")]), 64
        )
        xy = np.array([[t["x"], t["y"]] for t in tables])
        centre = np.mean(xy, axis=0)
        for point in xy:
            self.assertLess(
                np.linalg.norm(xy - (2 * centre - point), axis=1).min(), 1e-6
            )

    def test_face_decodes_from_front(self):
        for p in self.world.findall("model"):
            if not p.get("name", "").startswith("target_"):
                continue
            ident = int(p.get("name").split("_")[1])
            image = np.full((180, 180), 255, "uint8")
            for v in p.findall("link/visual"):
                if not v.get("name", "").startswith("cell_"):
                    continue
                dx, y, z = map(float, v.findtext("pose").split()[:3])
                self.assertGreater(dx, 0)
                unit = float(v.findtext("geometry/box/size").split()[1])
                # Facing marker front (+X), camera image right is +Y.
                col = round(y / unit) + 3
                row = 3 - round(z / unit)
                image[20 + row * 20 : 40 + row * 20, 20 + col * 20 : 40 + col * 20] = 0
            d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
            if hasattr(cv2.aruco, "ArucoDetector"):
                _, ids, _ = cv2.aruco.ArucoDetector(d).detectMarkers(image)
            else:
                _, ids, _ = cv2.aruco.detectMarkers(image, d)
            self.assertEqual(ids.flatten().tolist(), [ident])

    def test_spawn(self):
        ref = json.loads((self.root / "reference.json").read_text())
        scene = json.loads((self.root / "scene.json").read_text())
        self.assertEqual(
            [scene["robot_spawn"][k] for k in ("x", "y", "yaw")], ref["start"]
        )
        self.assertEqual(ref["trace"][0], [0, *ref["start"]])


if __name__ == "__main__":
    unittest.main()
