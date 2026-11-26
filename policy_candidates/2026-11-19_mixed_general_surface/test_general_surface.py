"""Shape-agnostic trigger and safe-goal checks on synthetic maps."""
import unittest
from unittest.mock import patch

import numpy as np

from osp_mission.general_route import inspectable_route
from osp_mission.general_surface import GeneralSurfaceFollowup
from osp_mission.near_pass_inspection import NearPassInspection


class Grid:
    resolution = .1
    origin = (0., 0., 0.)

    def __init__(self):
        self.cells = np.zeros((80, 80), dtype=np.int8)
        # A large L-shaped object: its outline is deliberately not four faces.
        self.cells[29:38, 31:34] = 100
        self.cells[35:38, 31:43] = 100

    def index(self, x, y):
        return int(y/self.resolution), int(x/self.resolution)

    def inside(self, row, col):
        return 0 <= row < 80 and 0 <= col < 80

    def reachable(self, point, clearance):
        return np.ones_like(self.cells, dtype=bool)

    def line_clear(self, start, end):
        return True


class GeneralSurfaceTest(unittest.TestCase):
    def setUp(self):
        self.grid = Grid()
        self.policy = GeneralSurfaceFollowup()
        self.viewed = np.zeros((4, 80, 80), dtype=bool)
        self.glance = dict(reason='processed_frame_dwell', map_camera=(2.6, 3.3), started=40.)

    def choose(self, connected=True, **overrides):
        mask = np.full((80, 80), connected, dtype=bool)
        distance = np.zeros((80, 80), dtype=float)
        values = dict(grid=self.grid, costmap=self.grid, pose=(2.6, 2.7, 0.),
                      viewed=self.viewed, glance=self.glance, glance_key=0,
                      now=50., remaining=300., veto=lambda goal: False)
        values.update(overrides)
        with patch('osp_mission.general_surface.r32.nav_goal_reachability',
                   return_value=(mask, distance)):
            return self.policy.choose(**values)

    def test_general_route_includes_frontier_and_excludes_marker_work(self):
        self.assertTrue(inspectable_route(dict(x=1., y=2., features=np.zeros(5))))
        self.assertTrue(inspectable_route(dict(box_point=[1., 2.])))
        for key in ('approach_id', 'focus_id', 'print_point'):
            self.assertFalse(inspectable_route({key: 7}))

    def test_irregular_object_can_get_one_followup_after_brief_look(self):
        self.assertIsNone(self.choose(glance=None))
        goal = self.choose()
        self.assertIsNotNone(goal)
        self.assertEqual(goal['r32_kind'], 'general_surface_view')
        self.policy.dispatched(goal, 50.)
        self.assertIsNone(self.choose())

    def test_front_surface_can_trigger_a_short_dwell_without_box_confirmation(self):
        camera = (2.6, 3.3, 0.)
        look = NearPassInspection().choose(
            self.grid.cells, .1, (0., 0.), camera, camera[:2], self.viewed,
            199.255, 1.12, camera, 50., 50., 300.)
        self.assertIsNotNone(look)
        self.assertLess(abs(look['turn_rad']), .05)
        blocked = np.ones_like(self.viewed)
        self.assertIsNone(NearPassInspection().choose(
            self.grid.cells, .1, (0., 0.), camera, camera[:2], blocked,
            199.255, 1.12, camera, 50., 50., 300.))

    def test_unreachable_or_already_viewed_surface_is_rejected(self):
        self.assertIsNone(self.choose(connected=False))
        self.assertIsNone(self.choose(viewed=np.ones_like(self.viewed)))
        self.assertIsNone(self.choose(glance=dict(self.glance, reason='spin_rejected')))
        self.assertIsNone(self.choose(glance=dict(self.glance, started=0.)))


if __name__ == '__main__':
    unittest.main()
