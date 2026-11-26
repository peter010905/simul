"""Synthetic map checks: L returns are provisional, and routes remain bounded."""
import unittest
import numpy as np

from osp_mission import _r32_navigation_core as r32
from osp_mission.angular_box import AngularBox, CornerTrack, local_corners


def grid(cells):
    return r32.Grid(cells, .05, (-4., -4., 0.))


class AngularBoxTest(unittest.TestCase):
    def setUp(self):
        cells = np.zeros((160, 160), dtype=np.int8)
        cells[80, 80:104] = 100
        cells[80:104, 80] = 100
        self.grid = grid(cells)
        self.pose = (-1.6, -1.6, .7)
        self.views = AngularBox()

    def test_l_corner_forms_provisional_goal(self):
        corners = local_corners(self.grid, self.pose)
        self.assertTrue(corners)
        corner = min(corners, key=lambda item: np.linalg.norm(item[0]))[0]
        self.views.corner_tracks = [CornerTrack(corner, 20., hits=2)]
        goal = self.views.choose(self.grid, self.grid, self.pose, (0., 0., 0.), 30., {}, lambda _: False)
        self.assertIsNotNone(goal)
        self.assertEqual(goal['box_view_scope'], 'corner_hypothesis')
        self.assertGreater(np.linalg.norm([goal['x']-corner[0],goal['y']-corner[1]]), .7)

    def test_one_scan_or_blocked_map_does_not_navigate(self):
        corner = local_corners(self.grid, self.pose)[0][0]
        self.views.corner_tracks = [CornerTrack(corner, 20., hits=1)]
        self.assertIsNone(self.views.choose(self.grid, self.grid, self.pose, (0., 0., 0.), 30., {}, lambda _: False))
        self.views.corner_tracks[0].hits = 2
        blocked = grid(np.full_like(self.grid.cells, 100))
        self.assertIsNone(self.views.choose(self.grid, blocked, self.pose, (0., 0., 0.), 30., {}, lambda _: False))

    def test_straight_wall_is_not_corner(self):
        cells = np.zeros((160, 160), dtype=np.int8)
        cells[80, 80:104] = 100
        self.assertEqual(local_corners(grid(cells), self.pose), [])

    def test_sparse_route_looks_in_place_then_bounds_retries(self):
        pose = (-.8, -.8, 0.)
        corner = local_corners(self.grid, pose)[0][0]
        self.views.corner_tracks = [CornerTrack(corner, 20., hits=2)]
        cells = np.full_like(self.grid.cells, 100)
        cells[56:72, 56:72] = 0
        cost = grid(cells)
        goal = self.views.choose(self.grid, cost, pose, (0., 0., 0.), 30., {}, lambda _: False)
        self.assertEqual(goal['box_view_scope'], 'corner_glance')
        self.assertTrue(goal['stationary'])
        self.views.dispatched(goal, 30.)
        other = self.views.choose(self.grid, cost, pose, (0., 0., 0.), 40., {}, lambda _: False)
        self.assertIsNotNone(other)
        self.assertNotAlmostEqual(goal['face_angle_odom'], other['face_angle_odom'])
        self.views.dispatched(other, 40.)
        self.assertIsNone(self.views.choose(self.grid, cost, pose, (0., 0., 0.), 50., {}, lambda _: False))


if __name__ == '__main__':
    unittest.main()
