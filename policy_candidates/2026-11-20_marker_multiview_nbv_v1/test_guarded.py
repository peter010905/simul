import math
import unittest
from unittest.mock import patch
import numpy as np
from osp_mission._r32_navigation_core import Grid
from osp_mission.center_wall import CenterWall
from osp_mission.guarded_nbv import LocalMultiView

class Locality(unittest.TestCase):
    def setUp(self):
        self.scheduler=LocalMultiView()
        self.scheduler.camera=(400.,1.18,.04,0.,0.)
        self.grid=Grid(np.zeros((75,75),np.int8),.1,(0.,0.,0.))
        self.pose=(2.,2.,0.)
        self.viewed=np.zeros((4,75,75),bool)
        self.base=dict(x=2.15,y=2.1,yaw=0.,scan_angle=0.,surface_odom=[3.,2.1],normal_odom=math.pi,
                       surface_map=[3.,2.1],features=np.zeros(5),distance=.18,face_inspection=True,
                       r32_kind='interior_surface',stationary=False)
        self.faces=[]
        for p,n in [([3.,2.1],[-1.,0.]),([2.9,2.72],[0.,-1.])]:
            p,n=map(np.array,(p,n))
            k=self.scheduler.register(p,math.atan2(n[1],n[0]))
            self.faces.append((k,p,n,.18,'interior',False))

    def select(self,base):
        with patch.object(CenterWall,'decide',return_value=base),patch.object(self.scheduler,'geometry',return_value=self.faces):
            return self.scheduler.decide(self.grid,self.grid,self.pose,(0.,0.,0.),self.viewed,60.,600.,lambda g:False)

    def test_native_frontier_untouched(self):
        frontier=dict(r32_kind='interior_frontier')
        self.assertIs(self.select(frontier),frontier)

    def test_scan_only_frontier_delegates_to_flow(self):
        scan_only=dict(self.base,r32_kind='interior_frontier')
        for k in ('surface_map','surface_odom','normal_odom'):
            del scan_only[k]
        self.assertIsNone(self.select(scan_only))

    def test_rerank_within_original_corridor(self):
        goal=self.select(self.base)
        self.assertGreaterEqual(len(goal['nbv_gain']),2)
        self.assertLessEqual(math.dist((goal['x'],goal['y']),(self.base['x'],self.base['y'])),.33)
        self.assertEqual(goal['surface_odom'],self.base['surface_odom'])
        self.assertGreaterEqual(goal['clearance_m'],.32)

    def test_long_flat_side_contributes_once(self):
        k,p,n,w,kind,fitted=self.faces[0]
        self.faces.append((2,p+np.array([0.,.5]),n,.2,kind,fitted))
        self.assertEqual(len(self.scheduler.independent({k:.7,2:.8},self.faces)),1)

    def test_single_surface_without_additional_information_keeps_native_goal(self):
        self.faces=self.faces[:1]
        self.assertIs(self.select(self.base),self.base)

if __name__=='__main__':
    unittest.main()
