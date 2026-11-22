"""Small checks for the new wall reservation and its safety gates."""
import unittest
from osp_mission.visual_survey import VisualSurvey


class WallReservationTests(unittest.TestCase):
    def test_wall_reservation_is_bounded_and_respects_box_work(self):
        survey = VisualSurvey()
        self.assertFalse(survey.wall_due(64., 600., False, True))
        self.assertTrue(survey.wall_due(80., 600., False, True))
        self.assertFalse(survey.wall_due(80., 600., True, True))
        self.assertTrue(survey.wall_due(120., 600., True, True))
        survey.last_wall = 120.
        self.assertFalse(survey.wall_due(180., 600., False, True))
        self.assertTrue(survey.wall_due(200., 600., False, True))
        self.assertFalse(survey.wall_due(550., 600., False, False))
        self.assertFalse(survey.wall_due(570., 600., False, True))


if __name__ == '__main__':
    unittest.main()
