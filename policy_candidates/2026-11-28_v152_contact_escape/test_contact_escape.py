""".68 close-contact escape: picks the open side, turns, creeps 0.3 m, stops on hazards or obstacles."""
import math
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(os.environ.get('OSP_OVERLAY', Path(__file__).resolve().parent / 'overlay'))
sys.path.insert(0, str(ROOT))

from osp_mission.contact_escape import (ContactEscape, base_points, clear_ahead, nearest,  # noqa: E402
                                        open_heading)


def scene(leg=(.20, 0.), wall_y=None, n=720):
    """Range scan from a laser at the base centre: a desk leg, optional wall, open elsewhere (8 m)."""
    angles = -math.pi + np.arange(n) * (2 * math.pi / n)
    ranges = np.full(n, 8.)
    lx, ly = leg
    for i, a in enumerate(angles):
        d = np.array([math.cos(a), math.sin(a)])
        t = d @ np.array([lx, ly])
        if t > 0 and np.linalg.norm(np.array([lx, ly]) - t * d) < .03:
            ranges[i] = min(ranges[i], t)
        if wall_y is not None and abs(d[1]) > 1e-6 and wall_y / d[1] > 0:
            ranges[i] = min(ranges[i], wall_y / d[1])
    return base_points(ranges, -math.pi, 2 * math.pi / n, .15, 12., (0., 0., 0.))


def run(escape, points_fn, pose, hazard_at=None, limit=300):
    t = 100.
    reason = None
    for k in range(limit):
        cmd = escape.step(t, points_fn(pose), pose, hazard_at is not None and t >= hazard_at)
        if cmd is None:
            return escape.reason, pose
        vx, wz = cmd
        x, y, yaw = pose
        pose = (x + vx * .1 * math.cos(yaw), y + vx * .1 * math.sin(yaw), yaw + wz * .1)
        t += .1
    return reason, pose


def test_picks_side_away_from_the_leg_and_steps():
    points = scene(leg=(.20, 0.))
    assert nearest(points) < .25
    heading = open_heading(points)
    assert heading is not None and abs(math.remainder(heading, math.tau)) > math.radians(90)
    esc = ContactEscape()
    assert esc.start(100., points, (0., 0., 0.))
    # the leg stays at the world origin side: recompute points in the robot frame each step
    def world(pose):
        x, y, yaw = pose
        c, s = math.cos(-yaw), math.sin(-yaw)
        lx, ly = .20 - x, -y
        return scene(leg=(c * lx - s * ly, s * lx + c * ly))
    reason, pose = run(esc, world, (0., 0., 0.))
    assert reason == 'stepped' and math.dist(pose[:2], (.20, 0.)) > .45


def test_no_start_without_contact_or_inside_cooldown():
    esc = ContactEscape()
    assert not esc.start(100., scene(leg=(.6, 0.)), (0., 0., 0.))       # nothing close
    assert esc.start(100., scene(), (0., 0., 0.))
    esc.end(101., 'test')
    assert not esc.start(110., scene(), (0., 0., 0.))                   # 20 s cooldown
    assert esc.start(122., scene(), (0., 0., 0.))


def test_hazard_and_blocked_ahead_stop():
    esc = ContactEscape()
    esc.start(100., scene(), (0., 0., 0.))
    reason, _ = run(esc, lambda p: scene(), (0., 0., 0.), hazard_at=100.)
    assert reason == 'hazard'
    assert not clear_ahead(np.array([[.2, 0.]])) and clear_ahead(np.array([[-.2, 0.], [.2, .3]]))


def test_no_echo_does_not_beat_measured_open_space():
    # behind: no echo at all (inf); to the left: measured 3 m. The measured side must win.
    n = 720
    angles = -math.pi + np.arange(n) * (2 * math.pi / n)
    ranges = np.full(n, .5)
    ranges[np.abs(angles) > 2.6] = np.inf
    ranges[np.abs(angles - math.pi / 2) < .3] = 3.
    ranges[np.abs(angles) < .1] = .2
    points = base_points(ranges, -math.pi, 2 * math.pi / n, .15, 12., (0., 0., 0.))
    assert abs(open_heading(points) - math.pi / 2) < .3


if __name__ == '__main__':
    test_picks_side_away_from_the_leg_and_steps()
    test_no_start_without_contact_or_inside_cooldown()
    test_hazard_and_blocked_ahead_stop()
    test_no_echo_does_not_beat_measured_open_space()
    print('ok')
