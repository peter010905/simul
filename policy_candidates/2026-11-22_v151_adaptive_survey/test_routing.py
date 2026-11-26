from collections import deque
import math

import numpy as np
from osp_mission._r32_navigation_core import Grid
from osp_mission.deferred_views import DeferredViews
from osp_mission.survey_distance import path_distances


def reference(free, start, resolution):
    distances = np.full(free.shape, np.inf)
    distances[start] = 0.
    queue = deque([start])
    while queue:
        row, col = queue.popleft()
        for r, c in ((row-1, col), (row+1, col), (row, col-1), (row, col+1)):
            if (0 <= r < free.shape[0] and 0 <= c < free.shape[1]
                    and free[r, c] and np.isinf(distances[r, c])):
                distances[r, c] = distances[row, col] + resolution
                queue.append((r, c))
    return distances


def test_matches_parent_exactly_in_connected_disconnected_and_diagonal_layouts():
    rng = np.random.default_rng(358)
    for shape in ((1, 1), (2, 24), (34, 26), (70, 110)):
        for resolution in (.05, .13):
            for density in (.1, .28, .7):
                free = rng.random(shape) > density
                start = tuple(d//2 for d in shape)
                free[start] = True
                np.testing.assert_array_equal(path_distances(free, start, resolution),
                                              reference(free, start, resolution))


def test_deferred_face_needs_new_connected_view_and_a_different_goal():
    grid = Grid(np.zeros((90, 100), dtype=np.int8), .05, (0., 0., 0.))
    nav = np.ones(grid.cells.shape, dtype=bool)
    nav[:, 60:] = False
    clearance = np.ones(grid.cells.shape)
    distance = np.ones(grid.cells.shape)
    retry = DeferredViews()
    retry.update_geometry(grid, grid, nav, distance, clearance, (0., 0., 0.))
    pt = (2., 2.)
    for t in (10., 65.):
        retry.dispatched('wall', dict(x=2.8, y=2., surface_odom=pt, normal_odom=0.), t)
    def blocked(*args):
        return retry.view_filter('wall', pt, 0., .85, *args)
    assert blocked([10., 65.], 160., lambda g: False) is None
    nav[:, 60:] = True
    assert blocked([10., 65.], 70., lambda g: False) is None
    allowed = blocked([10., 65.], 160., lambda g: False)
    assert allowed and allowed(dict(x=2.81, y=2.))
    goal = dict(x=3.25, y=2.)
    assert not allowed(goal) and goal['reopened_face']
    assert blocked([10., 65.], 160., lambda g: True)(goal)
    assert blocked([10., 65., 160.], 270., lambda g: False) is None


def test_anonymous_print_prefers_shorter_metric_path_at_two_resolutions():
    from osp_mission.hybrid import PrintMemory

    for res in (.05, .1):
        grid = Grid(np.zeros((round(12/res), round(12/res)), dtype=np.int8), res, (0., 0., 0.))
        memory = PrintMemory()
        memory.tracks = [
            dict(stamp=5., count=3, point=np.array([7., 2.]), observer=np.array([2., 2.])),
            dict(stamp=5., count=3, point=np.array([2., 5.]), observer=np.array([2., 2.])),
        ]
        # Only the near candidate's longer standoff clears the local veto.
        veto = lambda g: g['print_point'][1] == 5. and g['y'] > 3.3
        goal = memory.choose(grid, grid, (2., 2., 0.), (0., 0., 0.), {}, 6., veto)
        assert goal['print_point'] == [2., 5.]
        assert math.isclose(goal['y'], 3.2)


def test_one_marker_on_a_box_keeps_unseen_faces_until_camera_coverage():
    from osp_mission.box_batch import BoxBatch
    batch = BoxBatch()
    center = (2.0, 1.0)
    goal = dict(surface_odom=[2.3, 1.], previous_attempts=0)
    batch.update([center], [(2.25, 1.)], (2.8, 1.), [(center, False)])
    batch.rank((1., goal))
    assert goal['tagged_box'] and not goal['completed_box']
    batch.update([center], [], (2.8, 1.), [(center, True)])
    batch.rank((1., goal))
    assert goal['completed_box']
    shifted = (2.05, 1.)
    batch.update([shifted], [], (2.8, 1.))
    batch.rank((1., goal))
    assert goal['completed_box']
    batch.update([shifted], [], (2.8, 1.), [(shifted, False)])
    batch.rank((1., goal))
    assert not goal['completed_box']
