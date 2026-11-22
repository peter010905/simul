"""Verify all four mapped sides receive a view opportunity."""
import math
import unittest
import numpy as np
from osp_mission._r32_navigation_core import Grid, connected_nav_cells
from osp_mission.visual_survey import VisualSurvey


class FourWallTests(unittest.TestCase):
    def test_four_reachable_cardinal_walls_rotate_without_repeating_a_side(self):
        cells = np.zeros((70, 100), dtype=np.int8)
        cells[[0, -1], :] = 100
        cells[:, [0, -1]] = 100
        grid = Grid(cells, .1, (0., 0., 0.))
        safe, distances = grid.reachable((5., 3.), .32)
        nav = connected_nav_cells(grid, (5., 3.))
        survey = VisualSurvey()
        survey.camera = (332., 1.25)
        walls = []
        for x, y, fx, fy, normal in ((.8, 3., .05, 3., 0.),
                                     (9.2, 3., 9.95, 3., math.pi),
                                     (5., .8, 5., .05, math.pi / 2),
                                     (5., 6.2, 5., 6.95, -math.pi / 2)):
            walls.append((0., dict(x=x, y=y, surface_map=[fx, fy],
                                   surface_odom=[fx, fy], normal_odom=normal, scan_angle=.4)))
        selected = []
        for elapsed in (80., 160., 240., 320.):
            goal = survey.choose(grid, grid, nav, safe, distances, (5., 3., 0.),
                                 (0., 0., 0.), walls, [], elapsed, 600.,
                                 lambda _: False, force_wall=True)
            self.assertIsNotNone(goal)
            selected.append(survey.cardinal_wall(goal))
            survey.dispatched(goal, elapsed)
        self.assertEqual(set(selected), {0, 1, 2, 3})


if __name__ == '__main__':
    unittest.main()
