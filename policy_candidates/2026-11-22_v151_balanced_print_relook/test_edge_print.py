import importlib.util
import json
import math
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

import cv2
import numpy as np

from osp_mission.edge_print import budget, camera_turn, clearance_at, edge_print
from osp_mission.marker_pair import decode


def grid(yaw=0.):
    orientation=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2))
    origin=NS(position=NS(x=0.,y=0.),orientation=orientation)
    return NS(info=NS(width=32,height=32,resolution=.05,origin=origin),data=[0]*1024)


def signal(ident=0):
    return dict(id=ident, stamp=15., pixels=12., origin_odom=[0.,0.], yaw_odom=.5)


class PrintTests(unittest.TestCase):
    def test_actual_original_dictionary_and_higher_quality_decode_remain_independent(self):
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        render=getattr(cv2.aruco,'generateImageMarker',None) or cv2.aruco.drawMarker
        for ident in (0,83,542,1023):
            for x,width,qualifies in ((450,14,True),(230,14,False),(420,35,False)):
                frame=np.full((480,480),230,np.uint8)
                frame[210:210+width,x:x+width]=render(dictionary,ident,width)
                native=[]
                corners,ids=decode(frame,dictionary,[],native)
                self.assertIn(ident,ids.ravel())
                result=edge_print(native,corners,480,480,18)
                self.assertEqual(result is not None,qualifies)
                if result:self.assertEqual(result[0],ident)

    def test_edge_crop_ambiguity_and_warped_only_results(self):
        quad=np.float32([[[455,200],[468,201],[468,218],[455,218]]])
        native=[(0,quad,13.)]
        self.assertIsNotNone(edge_print(native,[quad],480,480,18))
        self.assertIsNone(edge_print([], [quad],480,480,18))
        self.assertIsNone(edge_print(native, [quad,quad],480,480,18))
        self.assertIsNone(edge_print([(0,quad+np.float32([12,0]),13.)],[quad],480,480,18))
        self.assertIsNone(edge_print(native,[quad],480,480,12))

    def test_footprint_clearance_at_rotated_origin_and_unknown_cells(self):
        room=grid(math.pi/2)
        self.assertTrue(clearance_at(room,(-.8,.8)))
        room.data[16*32+20]=-1
        self.assertFalse(clearance_at(room,(-.8,.8)))
        self.assertFalse(clearance_at(room,(-.06,.4)))
        self.assertFalse(clearance_at(room,(math.nan,.4)))

    def test_exposure_and_latest_orientation_are_checked_separately(self):
        cue=signal()
        self.assertAlmostEqual(camera_turn(cue,15.1,(0.,0.,.1),.62),.4)
        for clock,camera in ((16.,(0.,0.,0.)),(15.1,(.5,0.,0.)),(15.1,(0.,0.,1.5)),
                             (15.1,(0.,0.,.5))):
            self.assertIsNone(camera_turn(cue,clock,camera,.62))

    def test_duplicate_false_cues_cannot_spend_the_mission(self):
        cue=signal()
        first=dict(source='edge_print',started=1.,candidate_id=0,odom_camera=[0,0,0],odom_yaw=.5)
        self.assertEqual(budget([],0.,15.,600.,585.,cue),8.)
        self.assertEqual(budget([first],3.,80.,600.,520.,cue),0.)
        self.assertEqual(budget([first],3.,80.,600.,520.,signal(93)),0.)
        far=dict(signal(93),origin_odom=[1.,0.])
        self.assertEqual(budget([first],3.,80.,600.,520.,far),8.)
        self.assertEqual(budget([first],3.,27.,600.,520.,far),0.)
        self.assertEqual(budget([first,first],4.,80.,600.,520.,far),0.)
        self.assertEqual(budget([],12.,80.,600.,520.,far),0.)
        self.assertEqual(budget([],0.,595.,600.,5.,far),0.)


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'run the ROS runtime check on Jazzy')
class RuntimeTests(unittest.TestCase):
    def rig(self):
        from builtin_interfaces.msg import Time
        from geometry_msgs.msg import TransformStamped
        from rclpy.time import Time as RTime
        from osp_mission.near_pass_runtime import NearPassRuntime
        from osp_mission.face_directed_look import FaceDirectedLook
        owner=Mock()
        clock=RTime.from_msg(Time(sec=15))
        owner.get_clock.return_value.now.return_value=clock
        owner.now.return_value=15.;owner.state='running';owner.edge_mode=True
        owner.kind='nav';owner.goal_handle=object();owner.pending=False;owner.cancel_reason=''
        owner.registered_pending=set();owner.targets={};owner.visual_ids=set();owner.readable_cue=None
        owner.marker_look.pending=None;owner.contact_pending=False;owner.recovery_pending=False
        owner.contact_inspection=NS(phase='',action='',pressed=False)
        owner.spin_blocked_until=0.;owner.started=0.;owner.inspection_camera=(340.,1.24)
        owner.inspection_camera_frame='optical';owner.marker_focus.hints={}
        owner.p.side_effect=lambda key: {'duration':600.,'map_frame':'map','odom_frame':'odom',
                                         'base_frame':'base_link'}[key]
        owner.active_goal={'x':5.,'face':2};owner.map=grid()
        owner.face_look=FaceDirectedLook(owner);owner.face_look.pending={'end_odom_yaw':1.}
        owner.camera_diagnostics={'accepted':10}
        def tf(parent,child,_):
            tr=TransformStamped();tr.header.stamp=Time(sec=15)
            tr.transform.rotation.w=1.
            if child=='optical':
                q=tr.transform.rotation;q.x=-.5;q.y=.5;q.z=-.5;q.w=.5
            else:tr.transform.translation.x=.8;tr.transform.translation.y=.8
            return tr
        owner.tf.lookup_transform.side_effect=tf
        runtime=NearPassRuntime(owner);runtime.event=Mock()
        runtime.print_hazard=False;runtime.hazard_received=15.
        return owner,runtime

    def test_face_route_resumes_after_safe_spin_and_actual_frame_dwell(self):
        owner,runtime=self.rig()
        runtime.receive_print(NS(data=json.dumps(signal())))
        self.assertTrue(runtime.request_print())
        self.assertEqual(owner.targets,{})
        owner.cancel_action.assert_called_once_with('near_pass_inspection')
        self.assertTrue(runtime.completed(5,'nav','near_pass_inspection'))
        self.assertTrue(runtime.tick())
        commanded=owner.send.call_args.args[1]
        self.assertAlmostEqual(commanded.target_yaw,.5)
        self.assertTrue(runtime.completed(4,'spin',''))
        self.assertTrue(runtime.tick())
        owner.now.return_value=16.;owner.camera_diagnostics['accepted']=14
        self.assertTrue(runtime.tick())
        self.assertEqual(owner.resume_action,('nav',owner.active_goal))
        self.assertIsNot(owner.resume_action[1],owner.active_goal)
        self.assertEqual(owner.face_look.pending,{'end_odom_yaw':1.})
        self.assertFalse(owner.targets)

    def test_hazards_protected_approaches_and_occupied_cells_never_cancel_a_goal(self):
        for mode in ('hazard','stale','obstacle','protected','registered','approach','recover'):
            owner,runtime=self.rig()
            if mode=='hazard':runtime.print_hazard=True
            if mode=='stale':runtime.hazard_received=10.
            if mode=='obstacle':owner.map.data[16*32+18]=100
            if mode=='protected':runtime.protected_navigation=True
            if mode=='registered':owner.targets={0:(1.,1.)}
            if mode=='approach':owner.marker_look.pending={'id':81}
            if mode=='recover':owner.recovery_pending=True
            runtime.receive_print(NS(data=json.dumps(signal())))
            self.assertFalse(runtime.request_print(),mode)
            owner.cancel_action.assert_not_called()

    def test_superior_ranged_evidence_after_dwell_returns_to_planner(self):
        owner,runtime=self.rig();runtime.receive_print(NS(data=json.dumps(signal(88))))
        self.assertTrue(runtime.request_print())
        self.assertTrue(runtime.completed(5,'nav','near_pass_inspection'))
        runtime.tick();runtime.completed(4,'spin','')
        owner.now.return_value=16.;owner.camera_diagnostics['accepted']=16
        owner.marker_focus.hints={88:{'stamp':15.8}}
        owner.resume_action=None
        self.assertTrue(runtime.tick());self.assertIsNone(runtime.task)
        self.assertIsNone(owner.resume_action)
        self.assertEqual(runtime.policy.history[-1]['reason'],'new_marker_evidence')


if __name__ == '__main__':
    unittest.main()
