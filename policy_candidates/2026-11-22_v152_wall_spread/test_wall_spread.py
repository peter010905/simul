"""Check that unknown regions can interrupt local surface goals safely."""
import math
import unittest

import numpy as np

from osp_mission._r32_navigation_core import Grid, connected_nav_cells
from osp_mission.center_wall import discovery_frontier
from osp_mission.visual_survey import VisualSurvey


class WallSpreadTests(unittest.TestCase):
    def setUp(self):
        cells=np.zeros((70,100),dtype=np.int8)
        cells[[0,-1],:]=100
        cells[:,[0,-1]]=100
        self.grid=Grid(cells,.1,(0.,0.,0.))
        self.safe,self.distances=self.grid.reachable((2.,3.),.32)
        self.nav=connected_nav_cells(self.grid,(2.,3.))

    @staticmethod
    def goal(x,y,unknown,surface):
        return dict(x=x,y=y,yaw=0.,stationary=False,distance=4.,
                    features=np.array([surface,unknown,.4,0.,0.]))

    def test_frontier_uses_actual_edge_frontier_gain(self):
        wrong=self.goal(3.,3.,0.,1.)
        frontier=self.goal(5.,3.,.8,0.)
        picked=discovery_frontier([wrong,frontier],self.grid,self.nav,lambda _:False)
        self.assertAlmostEqual(picked['x'],5.)

    def test_region_goal_is_safe_periodic_and_prefers_new_space(self):
        survey=VisualSurvey()
        survey.position(2.,3.,115.)
        target=self.goal(5.,3.,.8,0.)
        nearby=self.goal(2.4,3.,.9,0.)
        self.assertIsNone(survey.region_goal([target],self.grid,self.grid,self.nav,
            self.safe,self.distances,(0.,0.,0.),80.,600.,lambda _:False))
        selected=survey.region_goal([nearby,target],self.grid,self.grid,self.nav,
            self.safe,self.distances,(0.,0.,0.),120.,600.,lambda _:False)
        self.assertEqual(selected['x'],5.)
        self.assertTrue(selected['survey_region'])
        survey.dispatched(selected,120.)
        self.assertIsNone(survey.region_goal([target],self.grid,self.grid,self.nav,
            self.safe,self.distances,(0.,0.,0.),150.,600.,lambda _:False))
        self.nav[self.grid.index(5.,3.)]=False
        self.assertIsNone(survey.region_goal([target],self.grid,self.grid,self.nav,
            self.safe,self.distances,(0.,0.,0.),225.,600.,lambda _:False))

    def test_mapped_but_camera_unseen_surface_still_creates_region_task(self):
        survey=VisualSurvey()
        survey.position(2.,3.,115.)
        wall=self.goal(5.,3.,0.,1.5)
        chosen=survey.region_goal([wall],self.grid,self.grid,self.nav,
            self.safe,self.distances,(0.,0.,0.),120.,600.,lambda _:False)
        self.assertEqual(chosen['x'],5.)

    def test_wall_shortlist_includes_both_opposing_directions(self):
        survey=VisualSurvey()
        survey.camera=(332.,1.25)
        goals=[]
        for y in np.arange(.8,5.8,.5):
            goals.append((0.,dict(x=8.2,y=float(y),yaw=0.,surface_map=[8.95,float(y)],
                surface_odom=[8.95,float(y)],normal_odom=math.pi,scan_angle=.4)))
        goals.append((0.,dict(x=.8,y=3.,yaw=0.,surface_map=[.05,3.],
            surface_odom=[.05,3.],normal_odom=0.,scan_angle=.4)))
        considered=set()
        def veto(goal):
            considered.add(goal['survey_wall'][0])
            return False
        survey.choose(self.grid,self.grid,self.nav,self.safe,self.distances,
            (2.,3.,0.),(0.,0.,0.),goals,[],80.,600.,veto)
        self.assertEqual(considered,{0,4})


if __name__=='__main__':unittest.main()
