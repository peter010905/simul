from types import SimpleNamespace

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo

from osp_mission.detector import Detector
from osp_mission.marker_focus import MarkerFocus
from osp_mission.provisional_marker_cues import ProvisionalMarkerCues


def test_cue_needs_temporal_and_spatial_consistency():
    cue = ProvisionalMarkerCues()
    assert not cue.observe(42, [0., 2., .45], 1., 13.)
    assert not cue.observe(42, [0., 2., .45], 1.2, 13.)
    assert cue.observe(42, [0., 2., .45], 1.4, 13.)
    assert not cue.observe(42, [0., 2., .45], 1.4, 13.)
    assert not cue.observe(42, [0., 2., .45], 1.6, 13.)
    assert not cue.observe(42, [2., 2., .45], 3.5, 13.)
    assert not cue.observe(42, [2., 2., .45], 3.7, 13.)
    assert cue.observe(42, [2., 2., .45], 3.9, 13.)
    assert not cue.observe(1023, [2., 2., .45], 4., 2.)
    assert not cue.observe(42, [2., 2., .45], 4., 18.)


def test_repeated_provisional_hints_still_request_inspection():
    focus = MarkerFocus()
    for stamp in (1., 3., 5.):
        focus.observe(dict(id=42, stamp=stamp, map_point=[5., 4., .5],
                          observation_frame='map', provisional=True), [5., 2.], stamp)
    assert focus.request_interrupt(42, {}, 5.1)
    grid = np.zeros((100, 100), dtype=np.int8)
    selected = focus.choose(grid, .1, (0., 0.), (5., 2.), 0., [], {}, 6.1)
    assert selected is not None and selected['focus_id'] == 42
    assert not focus.request_interrupt(42, {}, 6.1)
    focus.observe(dict(id=42, stamp=7., map_point=[5., 4., .5],
                       observation_frame='map', provisional=False), [5., 2.], 7.)
    assert focus.hints[42]['count'] == 1
    assert focus.hints[42]['provisional'] is False


def test_small_image_candidates_never_count_as_recognized_ids():
    rclpy.init()
    node = Detector()
    try:
        tr = TransformStamped()
        tr.transform.rotation.x = -2**-.5
        tr.transform.rotation.w = 2**-.5
        identity = TransformStamped()
        identity.transform.rotation.w = 1.
        node.tf = SimpleNamespace(lookup_transform=lambda target, source, *a, **kw:
                                 tr if source == 'camera' else identity)
        info = CameraInfo()
        info.width, info.height = 640, 480
        info.k = [500., 0., 320., 0., 500., 240., 0., 0., 1.]
        info.d = [0.]*5
        node.on_info(info)
        hints, recognized = [], []
        node.candidate_pub = SimpleNamespace(publish=hints.append)
        node.pub = SimpleNamespace(publish=recognized.append)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        bridge = CvBridge()

        def frame(size, stamp):
            image = np.full((480, 640), 255, dtype=np.uint8)
            image[240-size//2:240+size//2, 320-size//2:320+size//2] = cv2.aruco.drawMarker(dictionary, 42, size)
            rgb = bridge.cv2_to_imgmsg(image, 'mono8')
            rgb.header.frame_id = 'camera'
            rgb.header.stamp.sec = int(stamp)
            rgb.header.stamp.nanosec = int(round((stamp-int(stamp))*1e9))
            depth = bridge.cv2_to_imgmsg(np.full((480, 640), 2., dtype=np.float32), '32FC1')
            depth.header = rgb.header
            node.on_depth(depth)
            node.on_rgb(rgb)

        for stamp in (1., 1.2, 1.4):
            frame(14, stamp)
        assert len(hints) == 1
        assert not node.observations
        node.publish()
        assert not [m for m in recognized[-1].markers if m.ns == 'point']
        for stamp in (2., 2.2, 2.4):
            frame(24, stamp)
        assert len(node.observations[42]) == 3
        node.publish()
        assert [m.id for m in recognized[-1].markers if m.ns == 'point'] == [42]
        # A horizontal tiny pattern must not produce an inspection cue.
        node.tf = SimpleNamespace(lookup_transform=lambda *a, **kw: identity)
        before = len(hints)
        for stamp in (4., 4.2, 4.4):
            frame(14, stamp)
        assert len(hints) == before
    finally:
        node.destroy_node()
        rclpy.shutdown()
