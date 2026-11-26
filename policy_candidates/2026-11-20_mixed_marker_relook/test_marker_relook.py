import math
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
from osp_mission.marker_relook import VisualRange
from osp_mission.mission import Mission


def track(memory, ident, start, x, target):
    origin = np.array([x, 0., 0.])
    ray = np.array(target) - origin
    distance = np.linalg.norm(ray)
    return [memory.observe(ident, start+dt, origin, ray/distance, 35/distance)
            for dt in (0., .1, .2, .3)]


def test_single_view_wrong_id_and_parallel_rays_do_not_locate_tag():
    memory = VisualRange()
    assert all(p is None for p in track(memory, 7, 1., 0., (0.,1.,0.)))
    assert all(p is None for p in track(memory, 8, 4., .5, (0.,1.,0.)))
    assert all(p is None for p in track(memory, 7, 6., .5, (.5,1.,0.)))
    points = track(VisualRange(), 7, 1., 0., (0.,1.,0.))
    assert points[-1] is None


def test_independent_views_recover_consistent_point():
    memory = VisualRange()
    track(memory, 7, 1., 0., (0.,1.,0.))
    assert np.allclose(track(memory,7,4.,.5,(0.,1.,0.))[-1], [0.,1.,0.])


def owner():
    q=NS(x=.5,y=.5,z=.5,w=.5)  # Optical +Z along odometry +X.
    tf=Mock()
    tf.lookup_transform.return_value.transform=NS(translation=NS(x=0,y=0),rotation=q)
    return NS(readable_cue=dict(id=5,stamp=10.,yaw_odom=.25,origin_odom=[0,0]),
              get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_100_000_000)),
              state='running',ready=lambda:True,targets={}, spin_blocked_until=0.,
              recovery_pending=False,contact_pending=False,near_pass=NS(task=None),
              contact_inspection=NS(phase='',pressed=False,action=''),
              marker_look=Mock(pending=None),face_look=Mock(pending={'end_odom_yaw':.7}),
              goal_handle=object(),pending=False,cancel_reason='',kind='spin',
              readable_turn=False,tf=tf,inspection_camera_frame='camera',
              send=Mock(),spin=object(),get_logger=Mock(),zero=Mock())


def test_face_turn_yields_once_and_marker_turn_waits_for_cancel_result():
    node=owner()
    node.cancel_action=lambda reason:setattr(node,'cancel_reason',reason)
    assert Mission.follow_readable_cue(node,10.)
    assert node.cancel_reason=='readable_marker'
    node.send.assert_not_called()
    node.goal_handle=None
    assert Mission.follow_readable_cue(node,10.2)
    assert math.isclose(node.send.call_args.args[1].target_yaw,.25)
    assert node.readable_cue is None


def test_contact_and_known_approach_take_precedence():
    for field, value in [('recovery_pending',True),('contact_pending',True),
                         ('batch_approach_id',42)]:
        node=owner()
        node.targets={42:(0,0)}
        setattr(node,field,value)
        assert not Mission.follow_readable_cue(node,10.)
        node.send.assert_not_called()
