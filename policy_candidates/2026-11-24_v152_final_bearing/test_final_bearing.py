import numpy as np

from osp_mission._r32_navigation_core import Grid
from osp_mission.bearing_advance import BearingAdvance


def test_only_successfully_registered_ids_can_release_bounded_budget():
    work = BearingAdvance()
    work.attempts = [(i, 1., (0., 0.), 12.) for i in (2, 2, 6, 6)]
    assert work.mission_limit(set(), 3) == 4
    assert work.mission_limit({2, 6}, 0) == 4
    assert work.mission_limit({2, 20}, 3) == 5
    assert work.mission_limit({20, 21}, 3) == 4
    assert work.mission_limit({2, 6}, 3) == 6
    assert work.mission_limit({2, 6, 102}, 3) == 4


def test_last_unregistered_bearing_uses_existing_route_and_reobservation_checks():
    work = BearingAdvance()
    work.attempts = [(i, 1., (0., 0.), 12.) for i in (2, 2, 6, 6)]
    grid = Grid(np.zeros((80, 90), np.int8), .05, (-2., -2., 0.))
    safe, walk = grid.reachable((0., 0.), .32)
    for stamp in (5., 5.2, 5.4):
        work.observe(dict(id=102, stamp=stamp, origin_odom=(0., 0.), yaw_odom=0., pixels=15.), stamp)
    def pick(known, veto=False):
        return work.choose(grid, grid, safe, safe, walk, (0., 0., 0.), (0., 0., 0.),
                           known, 5.5, 100., lambda g: veto, expected_count=3)
    assert pick({2, 6}, True) is None
    goal = pick({2, 6})
    assert goal['bearing_attempt_number'] == 5 and goal['bearing_attempt_limit'] == 6
    assert goal['bearing_advance_id'] == 102 and 'approach_id' not in goal
    work.dispatched(goal)
    assert pick({2, 6}) is None
