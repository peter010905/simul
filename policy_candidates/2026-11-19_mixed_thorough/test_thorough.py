"""Regression checks for sparse border evidence and reachable box faces."""
import unittest

import numpy as np

from osp_mission import _r32_navigation_core as r32
from osp_mission.hybrid import Hybrid, PrintMemory
from osp_mission.two_views import Track


class ThoroughPolicyTest(unittest.TestCase):
    def setUp(self):
        cells = np.zeros((160, 160), dtype=np.int8)
        cells[76:84, 76:84] = 100
        self.grid = r32.Grid(cells, .05, (-4., -4., 0.))
        self.pose = (-1.7, 0., 0.)
        self.views = Hybrid().box_views
        self.views.tracks = [Track(np.array([0., 0.]), 20., confirmed=True)]

    def choose(self, costmap=None):
        return self.views.choose(self.grid, costmap or self.grid, self.pose,
                                 (0., 0., 0.), 30., {}, lambda _: False)

    def test_complete_ring_uses_full_observation(self):
        self.assertEqual(self.choose()['box_view_scope'], 'full_ring')

    def test_blocked_ring_uses_reachable_faces(self):
        blocked = self.grid.cells.copy()
        blocked[:, 92] = 100
        costmap = r32.Grid(blocked, .05, self.grid.origin)
        goal = self.choose(costmap)
        self.assertIsNotNone(goal)
        self.assertEqual(goal['box_view_scope'], 'reachable_face')

    def test_no_box_confirmation_or_path_gives_no_goal(self):
        self.views.tracks.clear()
        self.assertIsNone(self.choose())
        self.views.tracks = [Track(np.array([0., 0.]), 20., confirmed=True)]
        blocked = r32.Grid(np.full(self.grid.cells.shape, 100, dtype=np.int8),
                           .05, self.grid.origin)
        self.assertIsNone(self.choose(blocked))

    def test_jittered_print_revisit_from_other_side(self):
        memory = PrintMemory()
        for i, x in enumerate((0., .18, .24)):
            stamp = 20. + i * .2
            memory.observe({'stamp': stamp, 'point': [x, 0.],
                            'observer': [-2., 0.], 'size': 12.}, stamp)
        self.assertEqual(memory.tracks[0]['count'], 3)
        empty = r32.Grid(np.zeros_like(self.grid.cells), .05, self.grid.origin)
        goal = memory.choose(empty, empty, (1.8, 0., 0.),
                             (0., 0., 0.), {}, 25., lambda _: False)
        self.assertIsNotNone(goal)
        self.assertLess(goal['x'], 0.)


if __name__ == '__main__':
    unittest.main()
