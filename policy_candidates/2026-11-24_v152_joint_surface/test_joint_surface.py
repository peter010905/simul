import math
from types import SimpleNamespace
import unittest

import numpy as np

from joint_surface import JointSurface, View, connecting_distances, projected_coverage


class Grid:
    resolution = .2

    def __init__(self):
        self.cells = np.ones((50, 50), dtype=bool)
        self.occluded = set()

    def index(self, x, y):
        return int(y / self.resolution), int(x / self.resolution)

    def world(self, row, col):
        return (col + .5) * self.resolution, (row + .5) * self.resolution

    def inside(self, row, col):
        return 0 <= row < 50 and 0 <= col < 50

    def line_clear(self, start, end):
        return self.index(*end) not in self.occluded


def goal(x, y, face, kind, yaw=0., normal=math.pi):
    return dict(x=x, y=y, yaw=yaw, scan_angle=0., surface_map=face, surface_odom=face,
                normal_odom=normal, r32_kind=kind, previous_attempts=0)


class JointSurfaceTests(unittest.TestCase):
    def setUp(self):
        self.grid = Grid()
        self.nav = self.grid.cells.copy()
        self.distance = np.full(self.nav.shape, 8.)
        self.scheduler = JointSurface()
        self.survey = SimpleNamespace(camera=(330., 1.0), reference_m=.10, readable_px=18.,
                                     wall_key=lambda g: (4, *self.grid.index(*g['surface_map'])),
                                     wall_times={}, last_direction={})

    def choose(self, walls, interiors, elapsed=40, duration=600):
        return self.scheduler.choose(self.grid, self.grid, self.grid.cells, self.nav,
                                     self.distance, (0., 0., 0.), self.survey,
                                     [(0., x) for x in walls], [(0., x) for x in interiors],
                                     None, elapsed, duration)

    def test_wall_and_object_are_counted_once(self):
        box = goal(1., 1., (2., 1.), 'interior_surface')
        wall = goal(1., 1.3, (2., 1.2), 'wall_surface')
        box_view = View(box, ('surface', 1), (5, 5), 1., 1.)
        wall_view = View(wall, ('wall', 1), (6, 5), 1., 1.)
        args = (self.survey.camera, 0., .1, 18.)
        covered = projected_coverage(self.grid, box_view, [wall_view, wall_view], *args)
        self.assertEqual(covered, {box_view.key: 1., wall_view.key: .70})
        self.grid.occluded.add(self.grid.index(1.9, 1.2))
        self.assertEqual(projected_coverage(self.grid, box_view, [wall_view], *args),
                         {box_view.key: 1.})

    def test_route_blocked_by_nav_mask_despite_close_endpoints(self):
        a, b = (5, 5), (5, 7)
        self.nav[:, 6] = False
        routes = connecting_distances(self.grid, self.grid, self.grid.cells, self.nav, a, [b], 2.4)
        self.assertEqual(routes, {})
        self.nav[5, 6] = True
        routes = connecting_distances(self.grid, self.grid, self.grid.cells, self.nav, a, [b], 2.4)
        self.assertAlmostEqual(routes[b], .4)

    def test_local_mixed_group_beats_isolated_long_trip(self):
        box = goal(1.1, 1.1, (2.0, 1.1), 'interior_surface')
        wall = goal(1.5, 1.1, (1.5, .35), 'wall_surface', -math.pi/2, math.pi/2)
        distant = goal(7., 2., (8., 2.), 'wall_surface')
        self.distance[self.grid.index(1.1, 1.1)] = 1.0
        self.distance[self.grid.index(1.5, 1.1)] = 1.4
        picked = self.choose([wall, distant], [box])
        self.assertEqual((picked['x'], picked['y']), (1.1, 1.1))
        self.assertIsNotNone(picked['joint_next_key'])
        self.scheduler.dispatched(picked, 40)
        self.distance[self.grid.index(1.5, 1.1)] = .4
        follow = self.choose([wall, distant], [], elapsed=49)
        self.assertEqual(follow['r32_kind'], 'wall_surface')
        self.assertTrue(follow['joint_continuation'])
        self.assertIsNone(self.choose([distant], [], elapsed=595))


if __name__ == '__main__':
    unittest.main()
