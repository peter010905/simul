"""Camera observation during a box turn contributes to wall-zone coverage."""
import unittest
import numpy as np
from osp_mission._r32_navigation_core import Grid
from osp_mission.edge_policy import DIRECTIONS
from osp_mission.visual_survey import VisualSurvey


class WallCameraCoverageTests(unittest.TestCase):
    def test_dispatched_wall_goal_is_only_an_attempt(self):
        survey = VisualSurvey()
        key = (1, 2, 3)
        survey.wall_times[key] = (0., 0.)
        survey.dispatched({'survey_wall': key, 'normal_odom': 0.,
                           'surface_odom': np.array([1., 2.])}, 10.)
        self.assertEqual(survey.direction_count, {})
        self.assertEqual(len(survey.direction_sections[0]), 1)

    def test_incidental_camera_observation_counts_new_spaced_wall_zone(self):
        cells = np.zeros((100, 100), dtype=np.int8)
        cells[:, 0] = 100
        grid = Grid(cells, .1, (0., 0., 0.))
        viewed = np.zeros((4,) + cells.shape, dtype=bool)
        survey = VisualSurvey()
        # No wall-view goal has been dispatched; the camera turned for a box.
        for elapsed,row in ((10., 50), (20., 70)):
            viewed[1,row,0] = True
            samples = {('wall', 1, 1, row, 0): [(row, 0)]}
            survey.progress(samples, grid, viewed, (0., 0., 0.),
                            (2., row * .1, 0.), elapsed, DIRECTIONS)
        self.assertEqual(survey.direction_count[0], 2)
        self.assertEqual(survey.direction_sections.get(0, []), [])
        # Replaying the same confirmed wall cell never adds a third look.
        survey.progress(samples, grid, viewed, (0., 0., 0.),
                        (2., 7., 0.), 30., DIRECTIONS)
        self.assertEqual(survey.direction_count[0], 2)


if __name__ == '__main__':
    unittest.main()
