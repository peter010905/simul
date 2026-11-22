"""Check the geometry, competing priorities and temporary revisit behavior."""
import math
import unittest

import numpy as np
from osp_mission._r32_navigation_core import Grid, connected_nav_cells
from osp_mission.visual_survey import VisualSurvey, from_odom, odom


def test_room():
    data = np.zeros((65,90), dtype=np.int8)
    data[[0,-1],:] = 100
    data[:,[0,-1]] = 100
    grid = Grid(data,.1,(-.05,-.05,0.))
    pose = (3.0,3.0,0.)
    safe, distances = grid.reachable(pose[:2],.32)
    return grid, pose, safe, distances, connected_nav_cells(grid,pose)


def walls():
    result = []
    for y in (.8,1.6,2.4,3.2,4.0,4.8,5.6):
        result.append((0.,dict(x=.8,y=y,yaw=math.pi,r32_kind='wall_surface',
            surface_map=[.05,y],surface_odom=[.05,y],normal_odom=0.,features=np.zeros(5),scan_angle=.4)))
    return result


class SurveyTests(unittest.TestCase):
    def setUp(self):
        self.grid,self.pose,self.safe,self.distance,self.nav = test_room()
        self.s = VisualSurvey()
        self.s.camera = (332.,1.25)

    def select(self,t,wall=(),interior=(),veto=lambda g:False):
        return self.s.choose(self.grid,self.grid,self.nav,self.safe,self.distance,self.pose,
                             (0,0,0),wall,interior,t,600.,veto)

    def test_unseen_wall_gets_turn_even_with_interior_and_views_multiple_sections(self):
        self.assertIsNone(self.select(5.,walls(),[(1.,{})]))
        goal = self.select(67.,walls(),[(1.,{})])
        self.assertGreaterEqual(goal['survey_visible_sections'],2)
        self.assertGreater(goal['standoff_m'],1.25)
        self.s.dispatched(goal,67.)
        self.assertIsNone(self.select(76.,walls(),[(1.,{})]))
        self.assertTrue(self.nav[self.grid.index(goal['x'],goal['y'])])

    def test_retrace_needs_excursion_and_lack_of_new_camera_evidence(self):
        for j,x in enumerate([*np.arange(1.,4.,.26),*np.arange(3.9,1.,-.26)]):
            self.s.position(x,1.,j*1.7)
        elapsed=(len(self.s.trail)-1)*1.7
        self.assertTrue(self.s.repeated_route(elapsed))
        self.s.last_progress=elapsed-1.
        self.assertFalse(self.s.repeated_route(elapsed))
        other=VisualSurvey()
        for j in range(24):other.position(j*.23,1.,1.7*j)
        self.assertFalse(other.repeated_route(40.))

    def test_newly_seen_face_on_neighbor_has_one_turn_within_the_cluster(self):
        self.s.loop_center=np.array([3.,3.])
        self.s.loop_until=115.
        face=dict(surface_odom=np.array([3.,3.3]),fitted_face=3,previous_attempts=0)
        first=self.select(81.,[],[(1.,face)])
        self.assertTrue(first['survey_new_face'])
        self.s.dispatched(first,81.)
        self.assertIsNone(self.select(84.,[],[(1.,face)]))

    def test_distant_positions_only_choose_safe_waypoints_on_observed_side(self):
        e=dict(source='distant_native_multiview',stamp=129.,id=341,point_odom=[.2,3.,.3],observer_odom=[4.,3.,.3])
        self.assertTrue(self.s.printed(e,129.1,54.))
        self.assertFalse(self.s.printed(e,129.2,54.1))
        g=self.select(56.,walls())
        self.assertEqual(g['distant_hint_id'],341)
        self.assertGreater(g['x'],.2)
        self.assertTrue(self.safe[self.grid.index(g['x'],g['y'])])
        self.assertIsNone(self.select(56.,walls(),veto=lambda _:True))
        self.s.dispatched(g,56.)
        self.assertIsNone(self.select(57.))

    def test_stale_false_or_already_confirmed_hint_cannot_interrupt(self):
        e=dict(source='distant_native_multiview',stamp=60.,id=4,point_odom=[.2,3.,.3],observer_odom=[4.,3.,.3])
        self.assertFalse(self.s.printed(e,62.,60.))
        self.assertFalse(self.s.printed(dict(e,point_odom=[float('nan'),3.]),60.,60.))
        self.s.known={4}
        self.assertFalse(self.s.printed(e,60.,60.))

    def test_remote_destination_can_require_a_previously_traversed_corridor(self):
        self.s.loop_center=np.array([3.,3.])
        self.s.position(3.,3.,20.)
        other=dict(x=6.,y=3.,yaw=0.,stationary=False,distance=3.,features=np.array([.2,0,3.,0.,0.]))
        result=self.s.remote_view([other],self.grid,self.grid,self.nav,self.safe,self.distance,
                                  (0,0,0),24.,500.,lambda _:False)
        self.assertEqual(result['r32_kind'],'unvisited_region')
        self.nav[self.grid.index(6.,3.)]=False
        self.assertIsNone(self.s.remote_view([other],self.grid,self.grid,self.nav,self.safe,self.distance,
                                            (0,0,0),24.,500.,lambda _:False))

    def test_rigid_transform_round_trip(self):
        for yaw in (0.,1.9,-2.8):
            transform=(-3.1,.8,yaw)
            np.testing.assert_allclose(from_odom(odom([1.7,2.3],transform),transform),[1.7,2.3])

    def test_rotated_wall_normal_and_wide_only_view(self):
        rot = math.pi/6
        fields = np.full((1,1),-.7071), np.full((1,1),-.7071)
        actual=self.s.wall_normal(fields,0,0,1,0,rot)
        np.testing.assert_allclose(actual, (math.cos(rot+math.pi/4),math.sin(rot+math.pi/4)))
        for _,g in walls():self.nav[self.grid.index(g['x'],g['y'])] = False
        selected=self.select(65.,walls(),[(1.,{})])
        self.assertIsNotNone(selected)
        self.assertGreater(selected['standoff_m'],1.25)

    def test_no_observed_free_cells_no_goal_and_wall_is_never_marked_by_dispatch(self):
        self.safe[:]=False
        self.assertIsNone(self.select(90.,walls()))
        self.s.dispatched(dict(r32_kind='interior_surface'),90.)
        self.assertFalse(self.s.evidence)
