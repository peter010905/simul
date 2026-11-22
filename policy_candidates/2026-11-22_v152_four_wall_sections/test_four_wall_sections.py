"""Check oblique direction grouping and long-wall endpoint sampling."""
import math
import unittest
import numpy as np
from osp_mission._r32_navigation_core import Grid, connected_nav_cells
from osp_mission.visual_survey import VisualSurvey


class FourWallSectionTests(unittest.TestCase):
    def test_diagonal_normals_do_not_collapse_into_two_sides(self):
        survey = VisualSurvey()
        groups = [survey.cardinal_wall({'normal_odom': angle}) for angle in
                  (math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4)]
        self.assertEqual(groups, [1, 2, 3, 0])

    def test_long_wall_evaluates_both_ends(self):
        cells = np.zeros((100, 100), dtype=np.int8)
        cells[[0, -1], :] = 100
        cells[:, [0, -1]] = 100
        grid = Grid(cells, .1, (0., 0., 0.))
        safe, distances = grid.reachable((2., 5.), .32)
        nav = connected_nav_cells(grid, (2., 5.))
        survey = VisualSurvey()
        survey.camera = (332., 1.25)
        walls = [(0., dict(x=.8, y=float(y), surface_map=[.05, float(y)],
                           surface_odom=[.05, float(y)], normal_odom=0., scan_angle=.4))
                 for y in np.arange(.8, 9.2, .4)]
        evaluated = set()
        def veto(goal):
            evaluated.add(round(goal['surface_odom'][1], 1))
            return False
        survey.choose(grid, grid, nav, safe, distances, (2., 5., 0.),
                      (0., 0., 0.), walls, [], 120., 600., veto, force_wall=True)
        self.assertLess(min(evaluated), 1.5)
        self.assertGreater(max(evaluated), 8.)


if __name__ == '__main__':
    unittest.main()
