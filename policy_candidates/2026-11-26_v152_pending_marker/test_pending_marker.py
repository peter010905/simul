from types import SimpleNamespace

import cv2
import numpy as np
import pytest
from osp_mission.marker_focus import MarkerFocus
from osp_mission.marker_pair import decode
from osp_mission.provisional_marker_cues import ProvisionalMarkerCues
from osp_mission.mission import Mission


def cue():
    focus = MarkerFocus()
    assert focus.observe(dict(provisional=True, id=42, stamp=10., map_point=[3.,3.,.2],
                              observation_frame='map', min_edge_pixels=8.),
                         [1.,3.], 10.1)
    return focus


def test_native_pixel_floor_and_spatial_consensus():
    gate = ProvisionalMarkerCues()
    for t in (0., .2, .4):
        assert not gate.observe(42, [0,0,1], t, 6.99)
    assert not gate.observe(42, [0,0,1], 1., 7.)
    assert not gate.observe(42, [1,0,1], 1.2, 7.)
    assert not gate.observe(42, [1,0,1], 1.4, 7.)
    assert gate.observe(42, [1,0,1], 1.6, 7.)
    assert not gate.observe(42, [1,0,1], 1.6, 7.)


@pytest.mark.parametrize('ident', [7, 42, 103, 751])
def test_decoded_small_image_has_matching_original_pattern(ident):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    native = np.full((80,100), 235, np.uint8)
    native[35:45,50:60] = cv2.resize(cv2.aruco.drawMarker(dictionary, ident, 140),
                                      (10,10), interpolation=cv2.INTER_AREA)
    corners, ids = decode(native, dictionary)
    assert ids.ravel().tolist() == [ident]
    edge = np.linalg.norm(corners[0][0] - np.roll(corners[0][0], 1, axis=0), axis=1).min()
    assert 7. <= edge < 10.


def test_first_plan_during_handoff_already_has_a_focus():
    focus = cue()
    goal = focus.choose(np.zeros((80,80), np.int8), .1, (0.,0.), (1.,3.), 0., [], {}, 10.15)
    assert goal['focus_id'] == 42
    assert not focus.request_interrupt(42, {}, 10.6, deferred=True)


def test_busy_handoff_does_not_consume_interrupt():
    calls = []
    owner = SimpleNamespace(marker_focus=cue(), state='running', kind='nav', goal_handle=object(),
                            pending=False, cancel_reason='marker_hint', targets={}, zero=lambda: None,
                            cancel_action=calls.append, get_logger=lambda: SimpleNamespace(info=lambda _: None),
                            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=12_000_000_000)))
    def wake():
        return Mission.interrupt_for_hint(owner, owner.marker_focus.hints, deferred=True)
    assert not wake()
    owner.cancel_reason = ''
    owner.contact_pending = True
    assert not wake()
    owner.contact_pending = False
    owner.marker_look = SimpleNamespace(pending={'kind':'approach', 'id':7})
    assert not wake()
    owner.marker_look.pending = None
    assert wake() and not wake()
    assert calls == ['marker_hint']


def test_stale_or_registered_hint_cannot_trigger_wakeup():
    focus = cue()
    assert not focus.request_interrupt(42, {42}, 12., deferred=True)
    assert not focus.request_interrupt(42, {}, 18.1, deferred=True)


def test_tiny_images_publish_only_a_ranged_provisional_message():
    import json
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import CameraInfo
    from osp_mission.detector import Detector

    rclpy.init()
    node = Detector()
    try:
        camera = TransformStamped()
        camera.transform.rotation.x, camera.transform.rotation.w = -2**-.5, 2**-.5
        node.tf = SimpleNamespace(lookup_transform=lambda *args, **kwargs: camera)
        bridge = CvBridge()
        image = np.full((480,640),235,np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        image[235:245,315:325] = cv2.resize(cv2.aruco.drawMarker(dictionary,42,140),
                                            (10,10),interpolation=cv2.INTER_AREA)
        info = CameraInfo(width=640, height=480, k=[500.,0.,320.,0.,500.,240.,0.,0.,1.], d=[0.]*5)
        node.on_info(info)
        hints, observations, named_bearings = [], [], []
        node.candidate_pub = SimpleNamespace(publish=hints.append)
        node.observation_pub = SimpleNamespace(publish=observations.append)
        node.bearing_pub = SimpleNamespace(publish=named_bearings.append)
        for i in range(6):
            msg = bridge.cv2_to_imgmsg(image,'mono8')
            msg.header.frame_id = 'camera'
            msg.header.stamp.sec, step = divmod(i,5)
            msg.header.stamp.sec += 1
            msg.header.stamp.nanosec = step*200000000
            if i >= 3:
                depth = bridge.cv2_to_imgmsg(np.full(image.shape,2.,np.float32),'32FC1')
                depth.header = msg.header
                node.on_depth(depth)
            node.on_rgb(msg)
            if i < 5:
                assert not hints
        assert not named_bearings and not observations and not node.observations and not node.close_read_ids
        assert len(hints) == 1
        event = json.loads(hints[0].data)
        assert event['id'] == 42 and event['provisional'] and 7. <= event['min_edge_pixels'] < 10.
    finally:
        node.destroy_node()
        rclpy.shutdown()
