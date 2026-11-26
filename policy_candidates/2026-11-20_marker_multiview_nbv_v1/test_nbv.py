import math
import unittest
import numpy as np
from osp_mission._r32_navigation_core import Grid
from osp_mission.nbv_faces import MultiView
from osp_mission.nbv_rgb import RGBMemory
from osp_mission.marker_nbv import MarkerNBV,HOLD_S

class Behavior(unittest.TestCase):
    def setUp(self):
        self.data=np.zeros((75,75),np.int16)
        self.grid=Grid(self.data,.1,(0.,0.,0.))
        self.tf=(0.,0.,0.)
        self.rgb=RGBMemory()
        self.schedule=MultiView()
        self.face=(0,np.array([3.,2.]),np.array([-1.,0.]),.27,'interior',True)

    def test_quality(self):
        score=lambda p,h: self.schedule.quality(self.grid,np.array(p),h,400.,1.2,self.face)
        front=score((2.,2.),0.)
        self.assertGreater(front,score((1.,.9),.4))
        self.assertEqual(score((2.,2.),math.pi),0.)
        self.assertEqual(score((4.,2.),math.pi),0.)
        self.grid.cells[:,25:27]=-1
        self.assertEqual(score((2.,2.),0.),0.)

    def test_credit_exposure_only(self):
        k=self.schedule.register(self.face[1],math.pi)
        self.schedule.local=[self.face]
        self.schedule.dispatched(dict(surface_odom=[3.,2.],normal_odom=math.pi),1.)
        self.assertEqual(self.schedule.faces[k].best,0.)
        endpoints=np.array([[3.,2.],[3.,2.1],[3.,1.9]])
        self.schedule.observe(self.grid,(2.,2.,0.),endpoints,self.tf,400.,1.2,(0.,0.,0.))
        self.assertGreater(self.schedule.faces[k].best,.85)

    def hit(self,t,point=True):
        src=np.array([1.,2.])
        target=[3.8,2.8]
        e=dict(observer=src,point=target if point else None,bearing=.28,uv=(.03,.6),size=21,stamp=t)
        self.rgb.observe(e,t)

    def choose(self,t,veto=lambda g:False):
        return self.rgb.choose(self.grid,self.grid,(1.,2.,0.),self.tf,{},t,veto)

    def test_bearing_only_look_limits_loss(self):
        self.hit(1.,False)
        g=self.choose(1.)
        self.assertEqual(g['nbv_action'],'relook')
        self.rgb.dispatched(g,1.)
        self.assertIsNone(self.choose(2.))
        self.assertIsNone(self.choose(5.))
        self.assertTrue(self.rgb.tracks[0].lost)

    def test_measured_approach_and_resolve(self):
        for t in [1.,1.2,1.4,1.6]:
            self.hit(t)
        self.assertEqual(self.rgb.tracks[0].count,4)
        g=self.choose(1.6,lambda g:g['stationary'])
        self.assertEqual(g['nbv_action'],'approach')
        self.rgb.dispatched(g,1.6)
        self.assertIsNone(self.choose(1.8,lambda g:g['stationary']))
        self.rgb.reconcile({103:(3.8,2.8)},self.tf)
        self.assertTrue(self.rgb.tracks[0].resolved)
        self.assertIsNone(self.choose(3.8))

    def test_occupancy_rejects_approach(self):
        for t in [1.,1.2,1.4,1.6]:
            self.hit(t)
        self.grid.cells[:,:]=-1
        self.assertIsNone(self.choose(1.6))

    def test_multiface_same_pose_and_occlusion(self):
        self.grid.cells[32:40,30:38]=100
        self.grid.cells[32:40,40:48]=100
        self.schedule.camera=(400.,1.24,0.,0.,0.)
        viewed=np.zeros((4,)+self.grid.cells.shape,bool)
        g=self.schedule.decide(self.grid,self.grid,(3.8,2.,1.57),self.tf,viewed,60.,600.,lambda g:False)
        self.assertGreaterEqual(len(g['nbv_gain']),2)
        self.assertGreaterEqual(g['clearance_m'],.32)

    def test_min_hold(self):
        policy=MarkerNBV()
        policy.changed_at=4.
        from types import SimpleNamespace
        self.assertFalse(policy.interrupt_allowed(SimpleNamespace(now=lambda:4.+HOLD_S-.1)))
        self.assertTrue(policy.interrupt_allowed(SimpleNamespace(now=lambda:4.+HOLD_S)))

class Replanning(unittest.TestCase):
    def setUp(self):
        from types import SimpleNamespace as NS
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import OccupancyGrid
        from rclpy.time import Time
        tr=TransformStamped()
        tr.transform.rotation.w=1.
        m=OccupancyGrid()
        m.info.width=m.info.height=2
        m.data=[0]*4
        self.clock=10.
        self.owner=NS(now=lambda:self.clock, get_clock=lambda:NS(now=lambda:Time(seconds=self.clock)),
                      targets={},tf=NS(lookup_transform=lambda *args:tr), map=m,
                      known_dock_pose=lambda frame:None,p=lambda key:key)
        self.policy=MarkerNBV()
        self.policy.checked_revision=-1
        self.policy.changed_at=4.
        self.policy.current=dict(nbv_rgb=2,nbv_score=.55)
        self.other=dict(nbv_rgb=3,nbv_score=.6)
        self.policy.select=lambda *args,**kw:self.other

    def test_improvement_margin_and_no_duplicate(self):
        self.assertFalse(self.policy.should_interrupt(self.owner))
        self.clock+=2.
        self.policy.memory.changed('strengthened')
        self.other['nbv_score']=.85
        self.assertTrue(self.policy.should_interrupt(self.owner))
        self.clock+=2.
        self.policy.memory.changed('strengthened')
        self.other['nbv_rgb']=2
        self.assertFalse(self.policy.should_interrupt(self.owner))

    def test_confirmed_id_deferred_until_minimum_hold(self):
        self.owner.targets={7:(1.,2.)}
        self.policy.changed_at=9.
        self.assertFalse(self.policy.should_interrupt(self.owner))
        self.clock=13.
        self.assertTrue(self.policy.should_interrupt(self.owner))
        self.clock=15.
        self.assertFalse(self.policy.should_interrupt(self.owner))

    def test_lost_active_cue_releases_navigation(self):
        from osp_mission.nbv_rgb import Track
        self.other=None
        self.policy.memory.tracks=[Track(2,5.,np.zeros(2),0.)]
        self.assertTrue(self.policy.should_interrupt(self.owner))

if __name__=='__main__':
    unittest.main()
