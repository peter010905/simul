""".64: a lap stop interrupted before arrival is resumed once, in the current map frame."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / 'overlay'))

from osp_mission.wall_lap import WallLap  # noqa: E402

TRANSFORM = (2.0, -1.0, math.pi / 2)   # map -> odom


def to_odom(p, t=TRANSFORM):
    x, y, a = t
    return [x + math.cos(a) * p[0] - math.sin(a) * p[1], y + math.sin(a) * p[0] + math.cos(a) * p[1]]


def lap_goal(p):
    return dict(x=p[0], y=p[1], yaw=0., wall_lap=True, r32_kind='wall_lap', lap_pose_odom=to_odom(p),
                lap_normal_odom=0., lap_tangent_odom=0.)


def never(goal):
    return False


def test_interrupted_stop_is_resumed_once():
    lap = WallLap()
    lap.expire(100., 600.)
    lap.dispatched(lap_goal((1.0, 2.0)))
    lap.dispatched(dict(x=5., y=5., focus_id=304))          # marker follow-up interrupts
    lap.expire(110., 600.)
    goal = lap.resume((0., 0., 0.), TRANSFORM, 110., never)
    assert goal is not None and goal['lap_resumed'] and lap.reason == 'resume_interrupted_lap_stop'
    assert math.isclose(goal['x'], 1.0, abs_tol=1e-9) and math.isclose(goal['y'], 2.0, abs_tol=1e-9)
    assert lap.resume((0., 0., 0.), TRANSFORM, 111., never) is None     # only once
    lap.dispatched(goal)
    lap.dispatched(dict(x=5., y=5., focus_id=305))          # a resumed stop is not re-queued
    assert lap.pending is None


def test_reached_old_or_far_stops_are_not_resumed():
    lap = WallLap()
    lap.expire(100., 600.)
    lap.dispatched(lap_goal((1.0, 2.0)))
    lap.position(*to_odom((1.0, 2.0)), 101.)                 # arrived: credited
    lap.dispatched(dict(x=5., y=5., focus_id=304))
    assert lap.pending is None
    lap.dispatched(lap_goal((1.0, 2.0)))
    lap.dispatched(dict(x=5., y=5., focus_id=304))
    assert lap.resume((0., 0., 0.), TRANSFORM, 100. + 91., never) is None   # too old
    lap.dispatched(lap_goal((1.0, 2.0)))
    lap.dispatched(dict(x=5., y=5., focus_id=304))
    assert lap.resume((9., 9., 0.), TRANSFORM, 110., never) is None         # too far now
    lap.dispatched(lap_goal((1.0, 2.0)))
    lap.dispatched(dict(x=5., y=5., focus_id=304))
    assert lap.resume((0., 0., 0.), TRANSFORM, 110., lambda g: True) is None  # vetoed


if __name__ == '__main__':
    test_interrupted_stop_is_resumed_once()
    test_reached_old_or_far_stops_are_not_resumed()
    print('ok')
