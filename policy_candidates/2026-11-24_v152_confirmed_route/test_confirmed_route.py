import numpy as np

from osp_mission import _r32_navigation_core as r32
from osp_mission.marker_approach import MarkerApproach


def fixture():
    cells = np.zeros((60, 70), dtype=np.int8)
    cells[:34, 20:22] = 100
    grid = r32.Grid(cells, .1, (0., 0., 0.))
    pose = (1., 2., 0.)
    targets = {102: (4., 2.), 7: (2., 3.), 6: (5., 3.)}
    work = MarkerApproach(3)
    work.reconcile_targets(targets)
    work.range_witnesses = {7: True, 6: True}
    work.hints[102] = dict(observer=np.array([5., 2.]), point=np.array(targets[102]), stamp=10.)
    safe, distances = grid.reachable(pose[:2], .32)
    return work, grid, pose, targets, safe, distances


def test_connected_approach_uses_observed_side_round_obstacle_and_counts_only_dispatch():
    work, grid, pose, targets, safe, distances = fixture()
    def pick(now):
        return work.routed_approach(grid, grid, safe, r32.connected_nav_cells(grid, pose),
                                    distances, pose, targets, now, 150., lambda g: False)
    assert work.choose(grid.cells, .1, (0., 0.), pose[:2], 0., [], targets, 30.) is None
    first = pick(30.)
    assert first['approach_id'] == 102 and first['x'] > targets[102][0]
    assert first['routed_approach_m'] > np.hypot(first['x']-pose[0], first['y']-pose[1])
    assert work.routed_views == {} and not work.verified(102)
    work.routed_dispatched(first, 30.)
    assert pick(40.) is None
    second = pick(46.)
    assert first['routed_offset'] != second['routed_offset']
    work.routed_dispatched(second, 46.)
    assert pick(100.) is None
    work.reconcile_targets({7: targets[7], 6: targets[6]})
    assert 102 not in work.routed_views


def test_costmap_failed_goals_time_and_incomplete_roster_can_reject_route():
    work, grid, pose, targets, safe, distances = fixture()
    def pick(cost, roster, time=150., blocked=False):
        return work.routed_approach(grid, cost, safe, r32.connected_nav_cells(cost, pose),
                                    distances, pose, roster, 30., time, lambda g: blocked)
    cells = grid.cells.copy()
    cells[:, 20:22] = 100
    assert pick(r32.Grid(cells, .1, (0., 0., 0.)), targets) is None
    assert pick(grid, targets, blocked=True) is None
    assert pick(grid, targets, time=10.) is None
    assert pick(grid, {102: targets[102]}) is None
    unknown = grid.cells.copy()
    unknown[:, 20:22] = -1
    safe, distances = r32.Grid(unknown, .1, grid.origin).reachable(pose[:2], .32)
    assert pick(grid, targets) is None
