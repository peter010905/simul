"""Checks for the .59 open-direction escape (no ROS graph: fake scan, tf and action)."""
import math
from types import SimpleNamespace

import osp_mission.mission as mission


def fake(scan_ranges, angle_min=-math.pi, inc=None):
    inc = inc or 2 * math.pi / len(scan_ranges)
    sent = []
    q = SimpleNamespace(x=0., y=0., z=0., w=1.)
    tr = SimpleNamespace(transform=SimpleNamespace(rotation=q, translation=SimpleNamespace(x=1., y=2., z=0.)))
    obj = SimpleNamespace(
        now=lambda: 100., latest_scan=SimpleNamespace(ranges=scan_ranges, angle_min=angle_min, angle_increment=inc,
                                                      range_min=.1, range_max=12., header=SimpleNamespace(frame_id='laser')),
        tf=SimpleNamespace(lookup_transform=lambda a, b, t: tr), p=lambda k: k,
        get_logger=lambda: SimpleNamespace(warning=lambda *_: None),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None)),
        nav='nav', send=lambda client, goal, kind: sent.append(goal))
    return obj, sent


def test_steps_toward_the_open_side():
    n = 360
    ranges = [0.4] * n
    for i in range(70, 111):           # a 40 deg opening centred at index 90 (-90 deg)
        ranges[i] = 3.0
    obj, sent = fake(ranges)
    assert mission.Mission.open_escape(obj)
    goal = sent[0].pose.pose.position
    heading = math.atan2(goal.y - 2., goal.x - 1.)
    expected = -math.pi + 90 * 2 * math.pi / n
    assert abs(math.remainder(heading - expected, 2 * math.pi)) < math.radians(12)
    assert abs(math.hypot(goal.x - 1., goal.y - 2.) - .6) < 1e-6


def test_no_step_when_everything_is_close():
    obj, sent = fake([0.5] * 360)
    assert not mission.Mission.open_escape(obj)
    assert sent == []
