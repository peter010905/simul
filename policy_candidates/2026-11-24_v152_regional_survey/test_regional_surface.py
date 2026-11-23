import math
from types import SimpleNamespace
import unittest

import numpy as np

from osp_mission.regional_surface import RegionalSurface


class Grid:
    def __init__(self):
        self.safe = np.ones((12, 12), dtype=bool)

    def index(self, x, y):
        return int(y), int(x)

    def inside(self, row, col):
        return 0 <= row < 12 and 0 <= col < 12

    def line_clear(self, first, second):
        return True


def view(x, y, kind, normal=math.pi):
    return dict(x=x, y=y, yaw=0., scan_angle=0., surface_odom=(x + .7, y),
                surface_map=(x + .7, y), normal_odom=normal, r32_kind=kind)


class RegionalSurfaceTests(unittest.TestCase):
    def setUp(self):
        self.selector = RegionalSurface()
        self.grid = Grid()
        self.distances = np.full((12, 12), 8.)
        self.survey = SimpleNamespace(camera=(450., .9), reference_m=.10, readable_px=18.,
                                     wall_key=lambda g: (4, *self.grid.index(*g['surface_map'])),
                                     last_direction={})

    def choose(self, walls, objects, t=10, frontier=None):
        return self.selector.choose(self.grid, self.grid.safe, self.distances,
                                    (0., 0., 0.), self.survey,
                                    [(0., g) for g in walls], [(0., g) for g in objects],
                                    None, t, 600, frontier=frontier)

    def test_local_wall_follows_object_without_repeating_same_face(self):
        box = view(1., 1., 'interior_surface')
        local_wall = view(2., 1., 'wall_surface')
        far_wall = view(8., 8., 'wall_surface')
        self.distances[1, 1] = .30
        self.distances[1, 2] = 1.6
        picked = self.choose([], [box])
        self.selector.dispatched(picked, 10)
        self.selector.dispatched({'approach_id': 7}, 14)
        follow = self.choose([local_wall, far_wall], [box], 15)
        self.assertEqual(follow['r32_kind'], 'wall_surface')
        self.assertTrue(follow['joint_continuation'])
        self.assertEqual(follow['x'], 2.)
        self.selector.dispatched(follow, 15)
        next_area = self.choose([local_wall, far_wall], [box], 21)
        self.assertEqual(next_area['x'], 8.)
        self.assertFalse(next_area['joint_continuation'])

    def test_new_frontier_can_outrank_far_wall(self):
        far_wall = view(8., 8., 'wall_surface')
        frontier = dict(x=3., y=2., features=[0., 1., 1., 0., 0.],
                        r32_kind='interior_frontier')
        self.distances[2, 3] = 1.
        self.assertIs(self.choose([far_wall], [], frontier=frontier), frontier)
        self.grid.safe[2, 3] = False
        self.assertEqual(self.choose([far_wall], [], frontier=frontier)['x'], 8.)

    def test_group_time_expires_even_after_marker_interrupt(self):
        first = self.choose([], [view(1., 1., 'interior_surface')])
        self.selector.dispatched(first, 10)
        self.selector.dispatched({'focus_id': 7}, 25)
        self.assertTrue(self.selector.can_continue(64))
        self.assertFalse(self.selector.can_continue(65))


if __name__ == '__main__':
    unittest.main()
