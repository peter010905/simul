"""Run against the independently installed R2 stage."""
import importlib.util
import json
import math
from types import SimpleNamespace as NS
from unittest import TestCase, main, skipUnless
from unittest.mock import Mock

from osp_mission.edge_watch import EdgeWatch


def row(ray=1., size=8., pos=.85):
    return (ray,size,pos,0.)


class TestEdgeWatch(TestCase):
    def offer(self, watch, t, hit, origin=(0.,0.), age=0.):
        return watch.offer(t,t+age,origin,1.6,[hit])

    def test_two_outgoing_cues_one_sub_eight_yield_six_degrees_once(self):
        watch = EdgeWatch()
        self.assertIsNone(self.offer(watch,1.,row()))
        req = self.offer(watch,1.2,row(size=6.,pos=.97))
        self.assertAlmostEqual(math.degrees(req[3]),-6.)
        watch.commit(req)
        for t in (2.,45.):
            self.offer(watch,t,row())
            self.assertIsNone(self.offer(watch,t+.2,row(size=6.,pos=.97)))

    def test_late_inward_ambiguous_different_ray_or_origin_do_not_trigger(self):
        for first, second, kwargs in [
                ([row()],row(pos=.97), dict(age=1.)),
                ([row(pos=.97)],row(pos=.85),{}),
                ([row(),row(ray=1.02)],row(pos=.97),{}),
                ([row()],row(ray=1.13,pos=.97),{}),
                ([row()],row(pos=.97),dict(origin=(.3,0.))),
                ([row(size=7.)],row(size=7.,pos=.97),{})]:
            with self.subTest(first=first,second=second,kwargs=kwargs):
                w = EdgeWatch()
                w.offer(1.,1.,(0.,0.),1.6,first)
                self.assertIsNone(self.offer(w,1.2,second,**kwargs))

    def test_rejected_requests_do_not_spend_quota_or_leak_stale_pair(self):
        w = EdgeWatch()
        for t,x in [(1.,0.),(50.,0.),(100.,2.),(150.,4.)]:
            self.offer(w,t,row(),origin=(x,0.))
            req = self.offer(w,t+.2,row(pos=.97),origin=(x,0.))
            if t==150:
                self.assertIsNone(req)
            else:
                self.assertIsNotNone(req)
                if t>1:
                    w.commit(req)
        self.assertEqual(len(w.accepted),2)

    @skipUnless(importlib.util.find_spec('rclpy'), 'requires ROS message types')
    def test_mission_guards_preserve_registered_goal_and_hazard_priority(self):
        from std_msgs.msg import String
        from osp_mission.mission import Mission
        for mode in ('normal','registered','hazard','spin','deadline','protected'):
            with self.subTest(mode=mode):
                w = EdgeWatch()
                owner = NS(edge_watch=w,state='running',contact_pending=False,recovery_pending=False,
                           spin_blocked_until=0,marker_look=NS(pending=None),face_look=NS(pending=None),
                           contact_inspection=NS(pressed=False,action='',phase='',bearing=None),
                           hybrid=NS(schedule=NS(weak=Mock())),near_pass=NS(task=None,protected_navigation=mode=='protected'),
                           registered_pending={7} if mode=='registered' else set(),edge_hazard_active=mode=='hazard',
                           goal_handle=True,pending=False,kind='spin' if mode=='spin' else 'nav',readable_cue=None,
                           started=0.,cancel_reason='',p=lambda key:3. if mode=='deadline' else 600.,get_logger=Mock())
                owner.on_anonymous_glance=lambda msg: Mission.on_anonymous_glance(owner,msg)
                for t,pos in [(1.,.85),(1.2,.97)]:
                    owner.get_clock=lambda:NS(now=lambda:NS(nanoseconds=round(t*1e9)))
                    owner.now=lambda:t
                    e=dict(stamp=t,origin=[0,0],camera_yaw=1.6,bearings=[[1.,8.]],image_xy=[[pos,0.]])
                    Mission.on_small_wall(owner,String(data=json.dumps(e)))
                self.assertEqual(len(w.accepted),int(mode=='normal'))
                self.assertEqual(owner.contact_inspection.bearing is not None,mode=='normal')


if __name__=='__main__':
    main()
