"""Checks against the installed v1.5.2.42 candidate."""
from types import SimpleNamespace

import numpy as np
from nav_msgs.msg import OccupancyGrid

from osp_mission._r32_navigation_core import Grid, connected_nav_cells
from osp_mission.box_access import box_routes
from osp_mission.phased_hybrid import PhasedHybrid
from osp_mission.region_transit import RegionTransit, route_connected


def frontier(x, y):
    return dict(x=x, y=y, features=[0., .5, 2., 0., 0.])


def interrupted():
    transit = RegionTransit()
    transit.dispatched({'region_destination_odom': (4., 2.)}, 10., (1., 2.))
    transit.interrupt('marker_hint')
    transit.dispatched({'print_point': (1.5, 2.5)}, 20., (1.5, 2.))
    return transit


def test_new_local_wall_blocks_original_goal_but_safe_view_exists():
    grid = Grid(np.zeros((80, 80), np.int8), .10, (0., 0., 0.))
    local = Grid(grid.cells.copy(), grid.resolution, grid.origin)
    local.cells[:, 40] = 100
    pose, transform = (1.75, 2.), (0., 0., 0.)
    nav = connected_nav_cells(grid, pose)
    safe, _ = grid.reachable(pose, .32)
    _, distance = box_routes(grid, grid, nav, safe, pose, transform, ('map', local))
    original, alternative = frontier(4.05, 2.), frontier(3.35, 2.3)
    assert not route_connected(grid, distance, original)
    assert route_connected(grid, distance, alternative)
    transit = interrupted()
    visited = np.array([[1., 2.], [1.25, 2.], [1.5, 2.], [1.75, 2.]])
    assert transit.mode(30., pose, []) == 'resume'
    selected = transit.choose('resume', [original, alternative], grid, distance,
                              transform, pose, visited, 100., lambda _: False)
    assert (selected['x'], selected['y']) == (3.35, 2.3)
    assert transit.choose('resume', [original], grid, distance, transform,
                          pose, visited, 100., lambda _: False) is None


def test_original_frontier_preferred_unless_route_requires_large_detour():
    grid = Grid(np.zeros((80, 80), np.int8), .10, (0., 0., 0.))
    original, alternative = frontier(4., 2.), frontier(3.2, 2.6)
    distance = np.full(grid.cells.shape, np.inf)
    distance[grid.index(3.2, 2.6)] = 2.
    transit = interrupted()
    visited = np.array([[1., 2.], [1.2, 2.], [1.4, 2.], [3.7, 2.]])
    args = ('resume', [original, alternative], grid, distance, (0., 0., 0.), (1.5, 2.),
            visited, 100., lambda _: False)
    distance[grid.index(4., 2.)] = 2.5
    assert transit.choose(*args)['x'] == 4.
    distance[grid.index(4., 2.)] = 9.
    assert transit.choose(*args)['x'] == 3.2


def test_marker_action_result_does_not_clear_paused_frontier():
    transit = interrupted()
    transit.navigation_finished()
    assert transit.mode(30., (1.5, 2.), []) == 'resume'
    transit.dispatched({'region_destination_odom': (4., 2.), 'region_action': 'resume'}, 31., (1.5, 2.))
    transit.navigation_finished()
    assert transit.mode(32., (1.5, 2.), []) is None


def test_flow_fallback_cannot_reissue_a_disconnected_goal(monkeypatch):
    hybrid = PhasedHybrid()
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.header.stamp.sec = 1
    msg.info.resolution = .1
    msg.info.width = msg.info.height = 80
    msg.info.origin.orientation.w = 1.
    msg.data = [0] * 6400
    hybrid.drive.costmap = msg
    grid = np.zeros((80, 80), np.int8)
    hybrid.schedule.exploration_routes = np.full(grid.shape, np.inf)
    settings = {'map_frame': 'map', 'duration': 600.}
    owner = SimpleNamespace(map=msg, started=0., now=lambda: 1., targets={}, approach=None,
                            p=settings.__getitem__,
                            get_clock=lambda: SimpleNamespace(
                                now=lambda: SimpleNamespace(nanoseconds=10**9)))
    monkeypatch.setattr(hybrid.schedule, 'decide', lambda *args, **kwargs: None)
    goal = frontier(4., 2.)
    result = hybrid.select(owner, goal, grid, None, (1.5, 2., 0.), (0., 0., 0.), choices=[goal])
    assert result is None and hybrid.band_rejected_selected
