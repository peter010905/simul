"""Camera-only stops and short routes must respect observed geometry."""

import math

import numpy as np

from osp_mission.view_route_policy import choose, choose_greedy, options, scan_mask


def test_scan_interval_wraps_and_supports_signed_sweeps():
    bearings = np.radians([170, -170, 0, 90])
    positive = scan_mask(bearings, math.radians(140), math.radians(80))
    negative = scan_mask(bearings, math.radians(-140), math.radians(-80))
    np.testing.assert_array_equal(positive, negative)
    assert positive[0] and positive[1] and not positive[2]
    assert scan_mask(bearings, 0., 2 * math.pi).all()


def test_camera_can_inspect_other_face_without_relocating_or_using_hidden_space():
    grid = np.zeros((45, 65), dtype=np.int8)
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = 100
    grid[5:40, 30] = 100
    viewed = np.ones((4,) + grid.shape, dtype=bool)
    viewed[1, 5:40, 30] = False
    robot = (4.15, 2.15)
    goals, previous = options(grid, .1, (0., 0.), robot, heading=math.pi,
                               visits=[robot] * 5, viewed=viewed, return_paths=True)
    stationary = [g for g in goals if g['distance'] == 0]
    assert stationary and any(g['scan_angle'] == 0 for g in stationary)
    assert all(g['features'][0] > 0 for g in stationary)
    assert all(grid[g['cell']] == 0 and g['features'][0] > 0 for g in goals)
    assert previous is not None
    weights = [1., 1.2, .4, .8, .65]
    assert choose_greedy(goals, weights) in goals
    assert choose(goals, weights) in goals
    viewed[:] = True
    assert options(grid, .1, (0., 0.), robot, viewed=viewed)[0] == []


def test_occluded_face_does_not_add_camera_reward_on_wrong_side():
    grid = np.zeros((45, 65), dtype=np.int8)
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = 100
    grid[1:-1, 30] = 100
    viewed = np.ones((4,) + grid.shape, dtype=bool)
    viewed[1, 5:40, 30] = False
    assert options(grid, .1, (0., 0.), (2., 2.), viewed=viewed)[0] == []
