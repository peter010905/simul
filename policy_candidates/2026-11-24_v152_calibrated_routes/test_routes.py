"""Run on Jazzy after sourcing the calibrated-routes stage."""
from collections import deque
import itertools
import math

import numpy as np
import pytest

from osp_mission.grid_travel import travel_distances
from osp_mission.wall_bearings import WallBearings


def reference(mask, start, resolution):
    distances = np.full(mask.shape, np.inf)
    distances[start] = 0.
    queue = deque([start])
    while queue:
        r, c = queue.popleft()
        for rr, cc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
            if (0 <= rr < mask.shape[0] and 0 <= cc < mask.shape[1]
                    and mask[rr, cc] and np.isinf(distances[rr, cc])):
                distances[rr, cc] = resolution + distances[r, c]
                queue.append((rr, cc))
    return distances


def test_exact_travel():
    rng = np.random.default_rng(248)
    for shape, resolution, occupancy in itertools.product(
            [(1, 1), (1, 64), (48, 1), (41, 65)], [.05, .07], [.0, .19, .79, 1.]):
        mask = rng.random(shape) >= occupancy
        start = (shape[0] // 2, shape[1] // 2)
        mask[start] = True
        np.testing.assert_array_equal(travel_distances(mask, *start, resolution),
                                      reference(mask, start, resolution))


class VisibleMap:
    @staticmethod
    def line_clear(start, end):
        return True


@pytest.mark.parametrize('origin_x', [-25., 0., 150.])
@pytest.mark.parametrize('reverse', [True, False])
def test_anonymous_corroboration_independent_of_map_origin_and_track_order(origin_x, reverse):
    memory = WallBearings()
    origins = [(origin_x, -.4), (origin_x, .4)]
    target = np.array([origin_x + 3., 0.])
    for index, origin in enumerate(origins):
        angle = math.atan2(target[1] - origin[1], target[0] - origin[0])
        for delay in (0., .20, .40):
            stamp = 1. + index + delay
            memory.observe(stamp, stamp, origin, angle, 8.)
    if reverse:
        memory.tracks.reverse()
    wall = dict(x=origin_x + 2., y=0., wall_path_m=1.2, surface_odom=target, surface_map=target)
    def choose():
        return memory.choose([(25., wall)], VisibleMap(), (0., 0., 0.), [], 3., 45., (400., 1.2), .1)
    assert choose()['weak_wall']
    wall['wall_path_m'] = 2.5
    assert choose() is None
    wall['wall_path_m'] = 1.2
    memory.tracks.pop()
    assert choose() is None
