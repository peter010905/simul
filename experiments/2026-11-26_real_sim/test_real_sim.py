import copy
import json
from pathlib import Path
import unittest
from types import SimpleNamespace
from xml.dom.minidom import parseString

import numpy as np

from consensus import Consensus, scan_evidence
from profile import apply, build
from persistent_scene import Transforms


HERE = Path(__file__).resolve().parent


class PersistentSceneTests(unittest.TestCase):
    def test_repeated_map_or_stationary_scans_do_not_create_static_evidence(self):
        c = Consensus(np.ones((1, 1)), .05)
        for t in range(20):
            c.add(t, (0, 0, 0), [[True]], [[False]])
        self.assertEqual(len(c.views), 1)
        self.assertFalse(c.mask().any())

    def test_three_separated_views_and_span_are_required(self):
        c = Consensus(np.ones((1, 1)), .05)
        c.add(0, (0, 0, 0), [[True]], [[False]])
        c.add(3, (.3, 0, 0), [[True]], [[False]])
        self.assertFalse(c.mask().any())
        c.add(6, (.6, 0, 0), [[True]], [[False]])
        self.assertTrue(c.mask().all())

    def test_moving_person_trail_contradicted_by_free_rays_is_rejected(self):
        c = Consensus(np.ones((1, 1)), .05)
        for i in range(3):
            c.add(i*3, (i*.3, 0, 0), [[True]], [[False]])
        c.add(9, (.9, 0, 0), [[False]], [[True]])
        self.assertFalse(c.mask().any())

    def test_invalid_ray_is_neither_hit_nor_free(self):
        h, f = scan_evidence([[1, 0]], [0, 0], 0, [float('inf')], 0, .1, .1, 12, 5)
        self.assertFalse(h.any() or f.any())

    def test_5m_slam_does_not_delete_12m_raw_measurements(self):
        h, f = scan_evidence([[4, 0], [6, 0]], [0, 0], 0, [10], 0, .1, .1, 12, 5)
        self.assertEqual(f.tolist(), [True, False])
        self.assertFalse(h.any())


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.rows = json.loads((HERE/'audit.json').read_text())
        self.profile = build(self.rows, 5., [self.rows[-1]['run']])

    def test_measured_geometry_and_whole_run_holdout(self):
        p = self.profile
        self.assertEqual((p['camera']['width'], p['camera']['height']), (250, 250))
        self.assertEqual(p['lidar']['beams'], 720)
        self.assertEqual(p['lidar']['raw_max_range_m'], 12)
        self.assertEqual(p['slam_max_range_m'], 5)
        self.assertFalse(set(p['calibration_runs']) & set(p['validation_runs']))

    def test_holdout_values_cannot_change_fitted_profile(self):
        modified = copy.deepcopy(self.rows)
        modified[-1]['timing']['/scan']['median_hz'] = 2
        result = build(modified, 5., [modified[-1]['run']])
        self.assertEqual(result['lidar'], self.profile['lidar'])
        self.assertGreater(result['heldout_errors'][0]['scan_rate_error_percent'], 100)

    def test_sensor_xml_preserves_collision_and_optics(self):
        xml = '''<robot><collision name="body"/><sensor type="rgbd_camera"><update_rate>5</update_rate>
        <camera><horizontal_fov>1.1</horizontal_fov><image><width>640</width><height>480</height></image></camera></sensor>
        <sensor name="rplidar" type="gpu_ray"><update_rate>10</update_rate><ray><scan><horizontal><samples>640</samples>
        <min_angle>-3.14</min_angle><max_angle>3.14</max_angle></horizontal></scan>
        <range><min>0.15</min><max>16</max></range></ray></sensor>
        <sensor name="ir_intensity_front" type="gpu_ray"><update_rate>30</update_rate>
        <ray><range><min>0.01</min><max>0.5</max></range></ray></sensor></robot>'''
        doc = apply(parseString(xml), self.profile)
        self.assertIn('name="body"', doc.toxml())
        self.assertEqual(doc.getElementsByTagName('horizontal_fov')[0].firstChild.data, '1.1')
        self.assertEqual(doc.getElementsByTagName('max')[0].firstChild.data, '12.0')
        self.assertEqual(doc.getElementsByTagName('width')[0].firstChild.data, '250')
        self.assertEqual(doc.getElementsByTagName('max')[1].firstChild.data, '0.5')
        self.assertEqual(doc.getElementsByTagName('sensor')[2].getElementsByTagName('update_rate')[0].firstChild.data, '30')

    def test_missing_sensor_fails_closed(self):
        with self.assertRaises(ValueError):
            apply(parseString('<robot/>'), self.profile)

    def test_wrong_or_mixed_profile_is_rejected(self):
        with self.assertRaises(ValueError):
            build(self.rows, 20., [self.rows[-1]['run']])
        rows = copy.deepcopy(self.rows)
        rows[0]['cameras']['/mission/processed_camera'][0]['width'] = 640
        with self.assertRaises(ValueError):
            build(rows, 5., [rows[-1]['run']])


class TransformTests(unittest.TestCase):
    def test_missing_map_to_odom_does_not_fabricate_static_geometry(self):
        tf = Transforms()
        tf.finish()
        with self.assertRaises(ValueError):
            tf.lookup('map', 'lidar', 10.)

    def test_timestamped_interpolation_and_no_distant_extrapolation(self):
        tf = Transforms()
        a, b = np.eye(4), np.eye(4)
        b[0, 3] = .2
        tf.dynamic[('map', 'odom')] = [(0., a), (.2, b)]
        tf.static[('odom', 'lidar')] = np.eye(4)
        tf.finish()
        self.assertAlmostEqual(tf.lookup('map', 'lidar', .1)[0, 3], .1)
        with self.assertRaises(ValueError):
            tf.lookup('map', 'lidar', 2.)


if __name__ == '__main__':
    unittest.main()
