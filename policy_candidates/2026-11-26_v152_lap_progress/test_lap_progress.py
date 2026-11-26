import math

import numpy as np
from osp_mission._r32_navigation_core import Grid
from osp_mission.wall_lap import WallLap
from osp_mission.hybrid import PrintMemory


def corridor():
    cells = np.zeros((80, 200), np.int8)
    cells[:3] = cells[-3:] = 100
    cells[:, :3] = cells[:, -3:] = 100
    return Grid(cells, .05, (0., 0., 0.))


def entered(lap, pose, heading):
    lap.entry = tuple(pose[:2])
    lap.tangent_odom = heading
    return lap


def test_continuation_never_goes_back_against_lap_direction():
    g = corridor()
    pose = (5., 1.2, 0.)
    _, dist = g.reachable(pose[:2], .32)
    for heading in (0., math.pi):
        lap = entered(WallLap(), pose, heading)
        goal = lap.choose(g, dist, pose, (0., 0., 0.), 30., 600., lambda _: False)
        assert goal is not None
        along = (goal['x']-pose[0])*math.cos(heading)+(goal['y']-pose[1])*math.sin(heading)
        assert along >= -.30


def test_credited_stops_are_not_selected_again():
    g = corridor()
    pose = (5., 1.2, 0.)
    _, dist = g.reachable(pose[:2], .32)
    lap = entered(WallLap(), pose, 0.)
    first = lap.choose(g, dist, pose, (0., 0., 0.), 30., 600., lambda _: False)
    lap.visited.append(tuple(first['lap_pose_odom']))
    second = lap.choose(g, dist, pose, (0., 0., 0.), 32., 600., lambda _: False)
    assert second is None or math.dist(second['lap_pose_odom'], first['lap_pose_odom']) >= .8


def test_unavailable_continuation_hands_over_instead_of_idling():
    g = corridor()
    pose = (5., 1.2, 0.)
    _, dist = g.reachable(pose[:2], .32)
    lap = entered(WallLap(), pose, 0.)
    veto = lambda _: True
    assert lap.choose(g, dist, pose, (0., 0., 0.), 30., 600., veto) is None
    assert lap.phase == 'perimeter'
    assert lap.choose(g, dist, pose, (0., 0., 0.), 44., 600., veto) is None
    assert lap.phase == 'perimeter'
    lap.choose(g, dist, pose, (0., 0., 0.), 45.5, 600., veto)
    assert lap.phase == 'interior' and lap.reason == 'perimeter_continuation_unavailable'


def test_entry_wait_is_bounded():
    g = corridor()
    pose = (5., 1.2, 0.)
    _, dist = g.reachable(pose[:2], .32)
    lap = WallLap()
    veto = lambda _: True
    lap.choose(g, dist, pose, (0., 0., 0.), 10., 600., veto)
    lap.choose(g, dist, pose, (0., 0., 0.), 39., 600., veto)
    assert lap.phase == 'perimeter'
    lap.choose(g, dist, pose, (0., 0., 0.), 40.5, 600., veto)
    assert lap.phase == 'interior' and lap.reason == 'no_safe_perimeter_goal'


def test_success_resets_the_wait_timer():
    g = corridor()
    pose = (5., 1.2, 0.)
    _, dist = g.reachable(pose[:2], .32)
    lap = entered(WallLap(), pose, 0.)
    lap.choose(g, dist, pose, (0., 0., 0.), 30., 600., lambda _: True)
    assert lap.choose(g, dist, pose, (0., 0., 0.), 40., 600., lambda _: False) is not None
    lap.choose(g, dist, pose, (0., 0., 0.), 50., 600., lambda _: True)
    assert lap.phase == 'perimeter'


def test_white_leg_streak_pauses_and_marker_progress_resumes():
    m = PrintMemory()
    for k in range(3):
        assert m.white_allowed(100.+k, 0)
        m.white_dispatched(100.+k, 0)
    assert not m.white_allowed(120., 0)
    assert m.white_allowed(253., 0)
    m.white_dispatched(260., 0); m.white_dispatched(261., 0); m.white_dispatched(262., 0)
    assert not m.white_allowed(263., 0)
    # A newly registered marker re-enables white cues at once.
    assert m.white_allowed(264., 1)


def test_progress_between_looks_resets_streak():
    m = PrintMemory()
    m.white_dispatched(10., 0); m.white_dispatched(11., 0)
    m.white_dispatched(12., 1)
    assert m.white_allowed(13., 1)
    m.white_dispatched(14., 1)
    assert m.white_allowed(15., 1)


def test_arrival_heading_matches_the_next_travel_direction():
    cells = np.zeros((140, 180), np.int8)
    cells[:3] = cells[-3:] = 100
    cells[:, :3] = cells[:, -3:] = 100
    g = Grid(cells, .05, (0., 0., 0.))
    lap = WallLap()
    pose = (1.5, 3.5, 0.)
    t = 9.
    reversed_turns = 0
    previous_yaw = None
    for _ in range(8):
        _, dist = g.reachable(pose[:2], .32)
        goal = lap.choose(g, dist, pose, (0., 0., 0.), t, 600., lambda _: False)
        if goal is None:
            break
        lap.dispatched(goal)
        move = math.atan2(goal['y']-pose[1], goal['x']-pose[0])
        if previous_yaw is not None and abs(math.remainder(move-previous_yaw, 2*math.pi)) > 3*math.pi/4:
            reversed_turns += 1
        for k in range(1, 11):
            lap.position(pose[0]+(goal['x']-pose[0])*k/10, pose[1]+(goal['y']-pose[1])*k/10, t)
        t += goal['distance']/.2+2.
        previous_yaw = goal['yaw']
        pose = (goal['x'], goal['y'], goal['yaw'])
    assert previous_yaw is not None and reversed_turns == 0
