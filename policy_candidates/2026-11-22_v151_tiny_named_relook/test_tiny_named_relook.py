"""Checks for the opt-in candidate. Pure tests run on Mac; ROS tests run on Jazzy."""
import importlib.util
import json
import math
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np
from osp_mission.edge_watch import EdgeWatch
from osp_mission.marker_pair import decode
from osp_mission.tiny_named_relook import aligned_turn, TinyNamedRelook


def hit(ident=32, ray=.5, size=9., v=0.):
    return (ident,ray,size,v)


def offer(w,t,hits,*,origin=(0,0),yaw=0.,age=0.,ledger=(),known=()):
    return w.offer(t,t+age,origin,yaw,hits,known,ledger)


def scene(ident=32, strong=False):
    d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    img=np.full((240,240),220,np.uint8)
    def paste(id_,left,width,height):
        pattern=cv2.aruco.drawMarker(d,id_,140)
        img[110:110+height,left:left+width]=cv2.resize(pattern,(width,height),interpolation=cv2.INTER_AREA)
    paste(ident,60,10 if ident==32 else 9,9)
    if strong:paste(103,150,42,42)
    return img,d


class TestTinyNamedRelook(unittest.TestCase):
    def test_actual_small_decodes_do_not_enter_returned_ids(self):
        for ident in (7,32,103,1023):
            img,d=scene(ident); tiny=[]
            self.assertEqual(decode(img,d,[],tiny),([],None))
            self.assertEqual([h[0] for h in tiny],[ident])
        img,d=scene(strong=True);tiny=[]
        _,ids=decode(img,d,[],tiny)
        self.assertEqual(ids.ravel().tolist(),[103])
        self.assertEqual(tiny,[])

    def test_cluttered_frame_runs_at_most_five_extra_pattern_checks_and_cannot_steer(self):
        img,d=scene(); q=np.float32([[[60,110],[69,110],[69,119],[60,119]]])
        quads=[q+np.float32([n*12,0]) for n in range(10)]
        ids=np.array([[i] for i in range(10)])
        with patch.object(cv2.aruco,'detectMarkers',return_value=(quads,ids,[])), \
                patch('osp_mission.marker_pair.printed_pattern_matches',return_value=True) as check:
            tiny=[]
            self.assertEqual(decode(img,d,None,tiny),([],None))
            self.assertEqual(check.call_count,5)
            self.assertEqual(tiny,[])

    def test_duplicate_capture_cannot_confirm_but_new_consistent_capture_can(self):
        w=TinyNamedRelook()
        for t in (1.,1.,.95):
            self.assertIsNone(offer(w,t,[hit()]))
        ident,req=offer(w,1.6,[hit(ray=.515)],origin=(.13,.02))
        self.assertEqual(ident,32)
        self.assertAlmostEqual(math.degrees(req[3]),6.)
        self.assertLess(abs(.515-req[3]),abs(.515))

    def test_no_pair_if_ambiguous_in_one_or_across_frames(self):
        for first,second,third in [
            ([hit(),hit(ident=103)],[hit()],None),
            ([hit()],[hit(ident=103)],[hit()]),
            ([hit(),hit()],[hit()],None),
            ([hit()],[hit(ident=7)],None),
            ([hit()],[hit(ray=.7)],None),
        ]:
            w=TinyNamedRelook()
            self.assertIsNone(offer(w,1.,first))
            self.assertIsNone(offer(w,1.2,second))
            if third:self.assertIsNone(offer(w,1.4,third))

    def test_known_stale_moved_insufficient_or_already_centered_ignored(self):
        for a,b,kwargs,t in [
            (hit(),hit(),dict(age=.7),1.2),(hit(),hit(),dict(origin=(.5,0)),1.2),
            (hit(),hit(),dict(known={32}),1.2),(hit(),hit(),{},2.1),
            (hit(size=7.9),hit(),{},1.2),(hit(),hit(v=.15),{},1.2),
            (hit(ray=.04),hit(ray=.04),{},1.2)]:
            w=TinyNamedRelook();self.assertIsNone(offer(w,1.,[a]))
            self.assertIsNone(offer(w,t,[b],**kwargs))

    def test_shared_quota_and_two_way_direction_cooldown(self):
        w=TinyNamedRelook();watch=EdgeWatch();watch.commit((1.,(2.,1.),.5,.1))
        offer(w,10.,[hit()])
        self.assertIsNone(offer(w,10.6,[hit()],ledger=watch.accepted))
        offer(w,35.,[hit()])
        _,req=offer(w,35.6,[hit()],ledger=watch.accepted)
        watch.commit(req);w.last_glance=req[:3]
        self.assertTrue(w.near_pass_covered(40.,(0,0),.55))
        self.assertFalse(w.near_pass_covered(40.,(0,0),-.55))
        self.assertFalse(w.near_pass_covered(55.,(0,0),.55))
        offer(w,100.,[hit()])
        self.assertIsNone(offer(w,100.6,[hit()],ledger=watch.accepted))
        self.assertIsNone(aligned_turn(.5,0,.55))
        self.assertIsNone(aligned_turn(.5,0,.41))
        self.assertAlmostEqual(aligned_turn(-.5,0,-.1),-math.radians(6))

    @unittest.skipUnless(importlib.util.find_spec('rclpy'),'Jazzy transport types')
    def test_actual_detector_to_mission_never_registers_maps_or_requests_travel(self):
        from geometry_msgs.msg import TransformStamped
        from sensor_msgs.msg import CameraInfo,Image
        from osp_mission.detector import Detector
        from osp_mission.mission import Mission
        for mode in ('clear','registered','hazard','near_pass','protected','no_tf','stale_tf','near_pass_cooldown'):
            with self.subTest(mode=mode):
                img,d=scene()
                info=CameraInfo(width=240,height=240)
                info.k=[180.,0.,120.,0.,180.,120.,0.,0.,1.]
                info.d=[0.]*5
                tr=TransformStamped()
                q=tr.transform.rotation;q.x=-.5;q.y=.5;q.z=-.5;q.w=.5
                detector=NS(bridge=Mock(),dictionary=d,info=info,tf=Mock(),get_logger=Mock(),
                            small_wall_pub=Mock(),processed_camera=Mock(),print_pub=Mock(),print_glance=None,
                            diagnostics=dict(frames=0,decoded={},accepted=0,errors=0),observations={},
                            p={'odom_frame':'odom'}.get)
                detector.tf.lookup_transform.return_value=tr
                detector.bridge.imgmsg_to_cv2.return_value=img
                owner=NS(state='running',edge_watch=EdgeWatch(),tiny_named_relook=TinyNamedRelook(),
                         visual_ids=set(),targets={32:(0.,0.)} if mode=='registered' else {},
                         near_pass=NS(task='active' if mode=='near_pass' else None,protected_navigation=mode=='protected',
                                     policy=NS(last_started=1. if mode=='near_pass_cooldown' else -math.inf)),
                         contact_pending=False,recovery_pending=False,edge_hazard_active=mode=='hazard',tf=Mock(),
                         registered_pending=set(),goal_handle=True,pending=False,kind='nav',cancel_reason='',
                         readable_cue=None,spin_blocked_until=0.,marker_look=NS(pending=None),face_look=NS(pending=None),
                         contact_inspection=NS(phase='',pressed=False,action='',bearing=None),started=0.,
                         hybrid=NS(schedule=NS(weak=Mock())),get_logger=Mock(),p={'duration':600.,'odom_frame':'odom'}.get)
                owner.tf.lookup_transform.return_value=tr
                if mode=='no_tf':owner.tf.lookup_transform.side_effect=RuntimeError('no transform')
                owner.on_anonymous_glance=lambda m:Mission.on_anonymous_glance(owner,m)
                for t in (10.,10.6):
                    msg=Image(width=240,height=240);msg.header.frame_id='camera'
                    n=round(t*1e9);msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(n,10**9)
                    if mode!='stale_tf':tr.header.stamp=msg.header.stamp
                    owner.now=lambda:t+.02
                    owner.get_clock=lambda:NS(now=lambda:NS(nanoseconds=n+20_000_000))
                    Detector.on_rgb(detector,msg)
                    detector.small_wall_pub.publish.assert_called()
                    out=detector.small_wall_pub.publish.call_args.args[0]
                    payload=json.loads(out.data)
                    self.assertEqual([h[0] for h in payload['tiny_decodes']],[32])
                    self.assertEqual(payload['stamp'],t)
                    # The shared bearing channel carries separate anonymous and tiny lists.
                    Mission.on_small_wall(owner,out)
                self.assertFalse(detector.observations)
                self.assertFalse(detector.diagnostics['decoded'])
                self.assertEqual(detector.diagnostics['errors'],0)
                self.assertEqual(len(owner.edge_watch.accepted),int(mode=='clear'))
                self.assertFalse(owner.visual_ids)
                if mode=='clear':
                    self.assertEqual(owner.targets,{})
                    self.assertAlmostEqual(owner.contact_inspection.bearing[1],math.radians(6))
                else:self.assertIsNone(owner.contact_inspection.bearing)


if __name__=='__main__':unittest.main()
