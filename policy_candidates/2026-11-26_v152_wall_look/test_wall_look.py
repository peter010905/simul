"""Checks for the .55 wall-look candidate (no ROS node, no robot)."""
import math
import numpy as np
import sys
from pathlib import Path

from osp_mission.wall_lap import WallLap

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / '2026-11-26_v152_lap_progress'))


def test_lap_stops_face_the_wall_ahead():
    from lap_sim import room
    grid = room('plain')
    lap = WallLap()
    pose = (1.5, 3.5, 0.)
    seen = 0
    t = 9.
    for _ in range(6):
        safe, dist = grid.reachable(pose[:2], .32)
        goal = lap.choose(grid, dist, pose, (0., 0., 0.), t, 600., lambda _: False)
        t += 10.
        if goal is None:
            continue
        lap.dispatched(goal)
        travel = goal['lap_tangent_odom']
        normal = goal['lap_normal_odom']           # points into the room
        turn = math.remainder(goal['yaw'] - travel, 2 * math.pi)
        assert abs(abs(turn) - math.radians(45)) < 1e-6
        # turned toward the wall: the facing has a component against the room normal
        assert math.cos(goal['yaw'] - normal) < 0
        seen += 1
        pose = (goal['x'], goal['y'], goal['yaw'])
    assert seen >= 3


def test_no_reversed_arrival_heading_with_wall_look():
    """The .50 check with a 165 deg bound: a reversal (about 180 deg) still fails, while
    inside corners turn about 140-155 deg because each stop faces the wall by 45 deg."""
    from osp_mission._r32_navigation_core import Grid
    cells = np.zeros((140, 180), np.int8)
    cells[:3] = cells[-3:] = 100
    cells[:, :3] = cells[:, -3:] = 100
    g = Grid(cells, .05, (0., 0., 0.))
    lap = WallLap()
    pose = (1.5, 3.5, 0.)
    t = 9.
    previous = None
    worst = 0.
    for _ in range(8):
        _, dist = g.reachable(pose[:2], .32)
        goal = lap.choose(g, dist, pose, (0., 0., 0.), t, 600., lambda _: False)
        if goal is None:
            break
        lap.dispatched(goal)
        move = math.atan2(goal['y'] - pose[1], goal['x'] - pose[0])
        if previous is not None:
            worst = max(worst, abs(math.remainder(move - previous, 2 * math.pi)))
        for k in range(1, 11):
            lap.position(pose[0] + (goal['x'] - pose[0]) * k / 10, pose[1] + (goal['y'] - pose[1]) * k / 10, t)
        t += goal['distance'] / .2 + 2.
        previous = goal['yaw']
        pose = (goal['x'], goal['y'], goal['yaw'])
    assert previous is not None and worst < math.radians(165)
