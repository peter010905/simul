import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from survey import AdaptiveSurvey
from osp_mission import _r32_navigation_core as r32
from osp_mission.adaptive_faces import clearance_grid


class Grid:
    resolution = .1
    origin = (0., 0., 0.)

    def __init__(self):
        self.cells = np.zeros((80, 100), dtype=np.int8)

    def world(self, row, col):
        return np.array([(col + .5) * self.resolution, (row + .5) * self.resolution])

    def index(self, x, y):
        return math.floor(y / self.resolution), math.floor(x / self.resolution)

    def inside(self, r, c):
        return 0 <= r < 80 and 0 <= c < 100

    def line_clear(self, a, b):
        for p in np.linspace(a, b, 60)[:-2]:
            cell = self.index(*p)
            if not self.inside(*cell) or self.cells[cell] != 0:
                return False
        return True


def surface(kind, face, point, yaw):
    return dict(x=point[0], y=point[1], yaw=yaw, surface_map=face, surface_odom=face,
                normal_odom=math.atan2(point[1] - face[1], point[0] - face[0]),
                r32_kind=kind + '_surface', scan_angle=.5, features=np.zeros(5),
                previous_attempts=0)


class SurveyTest(unittest.TestCase):
    def setUp(self):
        self.survey = AdaptiveSurvey()
        self.survey.camera = (332.65, math.radians(70))
        self.grid = Grid()
        self.nav = np.ones(self.grid.cells.shape, dtype=bool)
        self.walk = np.ones(self.grid.cells.shape)
        self.clearance = np.ones(self.grid.cells.shape)
        self.viewed = np.zeros((4,) + self.grid.cells.shape, dtype=bool)
        self.pose = (1., 1., 0.)
        self.groups = {('wall', 1, 1, 1, 1): [(i, 30) for i in range(18, 24)],
                       ('interior', 2, 1, 1, 5): [(i, 50) for i in range(18, 24)]}
        self.wall = surface('wall', (3.1, 2.1), (3.8, 2.1), math.pi - .25)
        self.box = dict(surface('interior', (5.1, 2.1), (5.8, 2.1), math.pi - .25),
                        fitted_face=1, local_batch=True)
        self.frontier = dict(x=2., y=5., yaw=0., scan_angle=.5, features=[0., 1., 1., 0., 0.])
        self.schedule = SimpleNamespace(
            cluster=SimpleNamespace(crowded_centers=lambda _: [], local_fragment=lambda *args: False),
            weak=SimpleNamespace(choose=lambda *args: None), weak_known=[], weak_clock=0.)
        self.candidates = {'wall': [(1., self.wall)], 'interior': [(1., self.box)]}
        self.veto = lambda _: False

    def choose(self, elapsed=0., duration=600., frontiers=()):
        return self.survey.choose(self.schedule, self.grid, self.grid, self.nav, self.walk,
                                  self.clearance, self.pose, (0., 0., 0.), self.viewed, self.groups,
                                  self.candidates, frontiers, elapsed, duration, self.veto)

    def test_classes_with_safe_options_get_a_turn_and_low_density_wall_is_served(self):
        kinds = []
        for elapsed in (0., 35., 70., 110., 145., 180.):
            phase, reason, choice = self.choose(elapsed, frontiers=[self.frontier])
            kinds.append(choice['survey_kind'])
            self.survey.dispatched(choice, elapsed, phase)
        self.assertEqual(set(kinds), {'wall', 'interior', 'frontier'})

    def test_late_reserve_does_not_allow_fresh_boxes_to_starve_overdue_wall(self):
        self.survey.waiting = {'wall': 200., 'interior': 510.}
        phase, reason, goal = self.choose(526.)
        self.assertEqual((phase, reason), ('wall', 'overdue_camera_or_frontier'))
        self.assertGreater(goal['survey_wait_s'], 300.)

    def test_continuous_unavailability_does_not_accumulate_debt(self):
        self.survey.waiting['wall'] = 0.
        self.veto = lambda g: g.get('r32_kind') == 'wall_surface'
        self.assertEqual(self.choose(360.)[2]['survey_kind'], 'interior')
        self.assertNotIn('wall', self.survey.waiting)
        self.veto = lambda _: False
        self.choose(380.)
        self.assertEqual(self.survey.waiting['wall'], 380.)

    def test_unreachable_unknown_or_vetoed_alternate_is_excluded(self):
        self.candidates['interior'] = []
        origin = self.wall['x'], self.wall['y']
        self.nav[:] = False
        self.nav[self.grid.index(*origin)] = True
        self.assertEqual(self.choose()[2]['x'], origin[0])
        self.clearance[self.grid.index(*origin)] = .31
        self.assertIsNone(self.choose())
        self.nav[:] = True
        self.clearance[:] = 1.
        self.walk[:] = np.inf
        self.assertIsNone(self.choose())
        self.walk[:] = 1.
        self.veto = lambda g: True
        self.assertIsNone(self.choose())

    def test_nearly_expired_mission_and_absent_calibration(self):
        self.assertIsNone(self.choose(599.))
        self.survey.camera = None
        self.assertIsNone(self.choose())

    def test_actual_read_quality_opportunity_uses_fov_normal_occlusion_and_coverage(self):
        surfaces = self.survey.samples(self.grid, self.viewed, self.groups)
        wall, obstacle = self.survey.gains(self.wall, self.grid, surfaces, 1.25, 1.2)
        self.assertGreater(wall, 0.)
        self.assertEqual(obstacle, 0.)
        self.assertEqual(self.survey.gains(dict(self.wall, yaw=0.), self.grid, surfaces, 1.25, 1.2),
                         (0., 0.))
        self.assertEqual(self.survey.gains(self.wall, self.grid, surfaces, .4, 1.2), (0., 0.))
        self.grid.cells[:, 35] = -1
        self.assertEqual(self.survey.gains(self.wall, self.grid, surfaces, 1.25, 1.2), (0., 0.))
        self.grid.cells[:] = 0
        self.viewed[1, 18:24, 30] = True
        remaining = self.survey.samples(self.grid, self.viewed, self.groups)
        self.assertEqual(self.survey.gains(self.wall, self.grid, remaining, 1.25, 1.2), (0., 0.))

    def test_one_pose_has_both_wall_and_obstacle_opportunities(self):
        self.groups = {('wall', 1, 1, 1, 1): [(i, 30) for i in range(19, 22)],
                       ('interior', 2, 1, 1, 2): [(i, 30) for i in range(22, 25)]}
        gains = self.survey.gains(self.wall, self.grid,
                                  self.survey.samples(self.grid, self.viewed, self.groups), 1.25, 1.2)
        self.assertTrue(all(x > 0. for x in gains))
        self.assertFalse(self.viewed.any())

    def test_strong_weak_wall_cue_can_compete_with_confirmed_box_and_requires_nav(self):
        self.schedule.weak.choose = lambda *args: dict(self.wall, weak_wall=True)
        self.assertTrue(self.choose()[2]['weak_wall'])
        self.nav[self.grid.index(self.wall['x'], self.wall['y'])] = False
        self.assertFalse(self.choose()[2].get('weak_wall', False))

    def test_verified_box_only_does_not_reenter_candidate_scoring(self):
        self.candidates['wall'] = []
        self.candidates['interior'] = [(1., dict(self.box, completed_box=True))]
        self.assertIsNone(self.choose())

    def test_nominal_travel_time_is_metric_and_shorter_safe_route_wins(self):
        self.candidates['wall'] = []
        a = dict(self.box, x=1., y=1., surface_odom=(1., 1.), local_batch=False)
        b = dict(self.box, x=2., y=1., surface_odom=(2., 1.), local_batch=False)
        self.candidates['interior'] = [(1., a), (1., b)]
        self.groups.clear()
        self.walk[self.grid.index(1., 1.)] = 6.
        self.walk[self.grid.index(2., 1.)] = 2.
        self.assertEqual(self.choose()[2]['x'], 2.)
        self.assertAlmostEqual(self.choose()[2]['survey_travel_m'], 2.)

    def test_region_age_within_a_single_class_reaches_remote_low_gain_surface(self):
        self.candidates['interior'] = []
        near = self.wall
        far = dict(near, x=8., y=2., surface_map=(7.3, 2.), surface_odom=(7.3, 2.))
        self.candidates['wall'] = [(1., near), (8., far)]
        self.walk[:, 65:] = 8.
        self.survey.regions = {('wall', 1, 0): 285., ('wall', 3, 0): 0.}
        chosen = self.choose(300.)[2]
        self.assertEqual(chosen['surface_odom'], far['surface_odom'])
        self.assertAlmostEqual(chosen['survey_region_wait_s'], 300.)

    def test_reopened_face_preserves_prevalidated_viewpoint(self):
        self.candidates['interior'] = []
        self.groups = {}
        self.walk[:] = .1
        self.walk[self.grid.index(self.wall['x'], self.wall['y'])] = 5.
        self.assertGreater(self.choose()[2]['x'], self.wall['x'] + .15)
        self.wall['reopened_face'] = True
        self.assertEqual(self.choose()[2]['x'], self.wall['x'])

    def test_production_rotated_grid_blocks_route_through_wall(self):
        cells = np.zeros((50, 90), np.int8)
        cells[:, 45] = 100
        grid = r32.Grid(cells, .1, (2.3, -1., math.pi / 3))
        robot = grid.world(20, 12)
        behind = grid.world(24, 65)
        front = grid.world(25, 35)
        pose = (*robot, .2)
        nav = r32.connected_nav_cells(grid, pose)
        _, walk = grid.reachable(robot, .32)
        clearance = clearance_grid(grid)
        def option(point):
            goal = dict(self.frontier, x=point[0], y=point[1])
            return self.survey.option('frontier', goal, grid, grid, nav, walk,
                                      clearance, pose, (0., 0., 0.), 0., 100., lambda _: False)
        self.assertIsNone(option(behind))
        self.assertGreater(option(front).walk, 2.)


if __name__ == '__main__':
    unittest.main()
