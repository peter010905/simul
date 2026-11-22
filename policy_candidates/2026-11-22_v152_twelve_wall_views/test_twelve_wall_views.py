"""Check twelve-view scheduling and separation of wall turns."""
import math
import unittest
from osp_mission.visual_survey import VisualSurvey


class TwelveWallTests(unittest.TestCase):
    def test_quota_is_twelve_distinct_views_with_box_interleaving(self):
        survey = VisualSurvey()
        self.assertTrue(survey.wall_due(35., 600., True, True))
        for direction in range(4):
            theta = direction * math.pi / 2
            for offset in (0., 1.5, 3.):
                goal = {'survey_wall': [direction * 2, 0, 0],
                        'normal_odom': theta,
                        'surface_odom': [-math.sin(theta) * offset,
                                         math.cos(theta) * offset]}
                survey.wall_times[tuple(goal['survey_wall'])] = (0., -math.inf)
                survey.dispatched(goal, 35. + 35. * sum(survey.direction_count.values()))
        self.assertEqual(survey.direction_count, {0: 3, 1: 3, 2: 3, 3: 3})
        self.assertFalse(survey.wall_due(500., 600., True, True))

    def test_same_position_does_not_fill_wall_quota(self):
        survey = VisualSurvey()
        goal = {'survey_wall': [0, 0, 0], 'normal_odom': 0., 'surface_odom': [0., 0.]}
        survey.wall_times[(0, 0, 0)] = (0., -math.inf)
        survey.dispatched(goal, 35.)
        survey.dispatched(goal, 70.)
        self.assertEqual(survey.direction_count[0], 1)


if __name__ == '__main__':
    unittest.main()
