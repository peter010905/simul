"""Run with the candidate's staged osp_mission on PYTHONPATH."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import Mock
from types import SimpleNamespace
import cv2
import numpy as np

from osp_mission.marker_pair import decode, SeparateMarkers
from osp_mission.small_wall_cues import anonymous_center, fragment_candidates, usable_named_quad
from osp_mission.thorough_borders import borders


def paired_image():
    image = np.full((240, 240), 230, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    draw = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    image[80:122, 60:102] = draw(dictionary, 7, 42)
    # Distinct, adjacent printed square with an intentionally invalid payload.
    tile = np.zeros((7, 7), np.uint8)
    tile[1:6, 1:6] = np.indices((5, 5)).sum(axis=0) % 2 * 230
    image[80:122, 110:152] = cv2.resize(tile, (42, 42), interpolation=cv2.INTER_NEAREST)
    return image, dictionary


class VisionTests(unittest.TestCase):
    def test_known_id_and_adjacent_unreadable_outline(self):
        image, dictionary = paired_image()
        corners, ids = decode(image, dictionary, [])
        self.assertEqual(ids.flatten().tolist(), [7])
        outlines = [h for h in borders(image) if anonymous_center(h, corners)]
        self.assertEqual(len(outlines), 1)
        self.assertAlmostEqual(outlines[0][0], 130.5, delta=1.)

    def test_clipped_named_quad_falls_back_to_anonymous(self):
        edge = np.float32([[[2, 40], [24, 40], [24, 64], [2, 64]]])
        eligible = [quad for quad in [edge] if usable_named_quad(quad, 100, 100)]
        self.assertTrue(anonymous_center((13, 52, 22), eligible))
        self.assertFalse(anonymous_center((13, 52, 22), [edge]))
        self.assertTrue(anonymous_center((25, 52, 22), [edge]))

    def test_fragment_passes_original_pixel_gates_and_never_decodes(self):
        image = np.full((100, 100), 140, np.uint8)
        image[42:51, 39:45] = 0
        image[44:48, 40:42] = 140
        image[48:50, 42:45] = 140
        image[:, 45:49] = 43
        hits = fragment_candidates(image, [], [])
        self.assertTrue(any(abs(u-43) <= 1 and abs(v-46) <= 1 and 6 <= size < 10 and not named
                            for u, v, size, named in hits))
        self.assertEqual(fragment_candidates(image, hits, []), [])
        known = np.float32([[[35, 39], [50, 39], [50, 54], [35, 54]]])
        self.assertEqual(fragment_candidates(image, [], [known]), [])

    def test_empty_scene_and_solid_furniture_do_not_add_hints(self):
        image = np.full((100, 100), 140, np.uint8)
        image[:, 60:65] = 43
        image[30:40, 32:42] = 0
        self.assertEqual(fragment_candidates(image, [], []), [])

    @unittest.skipUnless(importlib.util.find_spec('rclpy'), 'requires Jazzy message types')
    def test_real_rgb_callback_processes_both_outputs(self):
        from geometry_msgs.msg import TransformStamped
        from sensor_msgs.msg import CameraInfo, Image
        from osp_mission.detector import Detector
        image, dictionary = paired_image()
        camera = CameraInfo(width=240, height=240)
        camera.k = [180., 0., 120., 0., 180., 120., 0., 0., 1.]
        camera.d = [0.] * 5
        msg = Image(width=240, height=240)
        msg.header.frame_id = 'camera'
        msg.header.stamp.sec = 10
        transform = TransformStamped()
        transform.transform.rotation.w = 1.
        n = SimpleNamespace(
            bridge=Mock(), info=camera, depth=None, dictionary=dictionary, tf=Mock(),
            observations={}, observation_pub=Mock(), print_glance=None,
            processed_camera=Mock(), print_pub=Mock(), separate_markers=SeparateMarkers(),
            diagnostics=dict(frames=0, decoded={}, accepted=0, errors=0), id_diagnostics={},
            p={'min_marker_pixels':18, 'confirmations':3, 'map_frame':'map', 'odom_frame':'odom'}.get,
            visual_sighting=lambda *args: None, lidar_point=lambda ray,*args: ray * 2.,
            get_logger=Mock())
        n.tf.lookup_transform.return_value = transform
        n.bridge.imgmsg_to_cv2.return_value = image
        n.observation_pub.get_subscription_count.return_value = 0
        Detector.on_rgb(n, msg)
        self.assertNotIn('marker_errors', n.diagnostics, n.diagnostics)
        self.assertEqual(n.diagnostics['errors'], 0, n.diagnostics)
        self.assertEqual(n.diagnostics['decoded'], {'7': 1})
        self.assertEqual(n.diagnostics['accepted'], 1)
        n.print_pub.publish.assert_called_once()
        n.processed_camera.publish.assert_called_once()


if __name__ == '__main__':
    unittest.main()
