"""A corroborated distant print gets checked before another generic face goal."""
import math
import unittest
from unittest.mock import patch

import numpy as np

from osp_mission._r32_navigation_core import Grid
from osp_mission.center_wall import CenterWall


class WeakWallPriorityTests(unittest.TestCase):
    def test_stable_print_precedes_local_box_continuation(self):
        cells = np.zeros((100, 100), dtype=np.int8)
        cells[:, 0] = 100
        cells[48:53, 48:53] = 100
        grid = Grid(cells, .1, (0., 0., 0.))
        policy = CenterWall()
        policy.survey.camera = (300., math.radians(70))
        policy.batch.update([], [], (5.05, 3.5))
        policy.batch.active = (5.05, 5.05)
        cue = {'r32_kind': 'wall_surface', 'weak_wall': True, 'x': 2., 'y': 5.}
        with patch.object(policy.weak, 'choose', return_value=cue) as choose:
            goal = policy.decide(grid, grid, (5.05, 3.5, 0.), np.array([0., 0., 0.]),
                                 np.zeros((4,) + cells.shape, dtype=bool), 200., 600.,
                                 lambda _: False)
        self.assertGreater(policy.counts['interior'], 0)
        self.assertGreater(policy.counts['wall'], 0)
        choose.assert_called_once()
        self.assertIs(goal, cue)
        self.assertEqual(policy.reason, 'persistent_small_wall_candidate')


if __name__ == '__main__':
    unittest.main()
