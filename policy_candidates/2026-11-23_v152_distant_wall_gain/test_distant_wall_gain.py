"""An unseen wall beyond a traveled corridor must remain selectable."""
import math
import unittest

import numpy as np

from osp_mission._r32_navigation_core import Grid
from osp_mission.visual_survey import VisualSurvey


class DistantWallGainTests(unittest.TestCase):
    def test_far_unseen_section_outscores_equally_unseen_near_revisit(self):
        cells = np.zeros((100, 150), dtype=np.int8)
        cells[:, 10] = 100
        cells[:, 90] = 100
        grid = Grid(cells, .1, (0., 0., 0.))
        safe, distances = grid.reachable((3., 5.), .32)
        survey = VisualSurvey()
        survey.camera = (300., math.radians(70))
        survey.trail.extend((t, x, 5.) for t, x in ((190., 2.8), (192., 3.), (194., 3.2)))

        def wall(face_x, normal, viewpoint_x):
            face = np.array([face_x, 5.05])
            point = (viewpoint_x, 5.05)
            return dict(x=point[0], y=point[1], distance=float(distances[grid.index(*point)]),
                        surface_odom=face, surface_map=face, normal_odom=normal,
                        scan_angle=math.radians(22), unseen_wall_m=.75)

        near = wall(1.05, 0., 2.95)
        far = wall(9.05, math.pi, 7.05)
        goal = survey.choose(grid, grid, safe, safe, distances, (3., 5., 0.),
                             np.array([0., 0., 0.]), [(0., near), (0., far)], [],
                             200., 600., lambda _: False)
        self.assertIsNotNone(goal)
        self.assertGreater(goal['x'], 6.)


if __name__ == '__main__':
    unittest.main()
