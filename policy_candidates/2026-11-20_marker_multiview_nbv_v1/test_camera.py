"""Exercise the actual contour -> stamped ROS JSON -> RGB memory -> Nav2 proposal boundary."""
import json
import unittest
from types import SimpleNamespace
import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from sensor_msgs.msg import Image,CameraInfo
from osp_mission._r32_navigation_core import Grid
from osp_mission.marker_pair import decode
from osp_mission.nbv_camera import publish
from osp_mission.nbv_rgb import RGBMemory

class PixelToLook(unittest.TestCase):
    def test_non_id_square_can_trigger_look_then_measured_approach(self):
        im=np.full((160,240),240,np.uint8)
        cells=np.zeros((7,7),np.uint8)
        cells[1:6,1:6]=((np.indices((5,5)).sum(axis=0)%2)*255).astype(np.uint8)
        im[58:93,167:202]=cv2.resize(cells,(35,35),interpolation=cv2.INTER_NEAREST)
        _,ids=decode(im,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL))
        self.assertIsNone(ids)
        msg=Image()
        msg.header.frame_id='camera_optical'
        msg.width,msg.height=240,160
        info=CameraInfo()
        info.width,info.height=240,160
        info.k=[180.,0.,120.,0.,180.,80.,0.,0.,1.]
        tr=TransformStamped()
        tr.transform.translation.x=2.
        tr.transform.translation.y=3.
        q=tr.transform.rotation
        q.x,q.y,q.z,q.w=-.5,.5,-.5,.5
        emitted=[]
        node=SimpleNamespace(info=info,p=lambda key:'odom',tf=SimpleNamespace(lookup_transform=lambda *args:tr),
                             print_pub=SimpleNamespace(publish=lambda packet:emitted.append(json.loads(packet.data))),
                             lidar_point=lambda ray,frame,stamp:None)
        memory=RGBMemory()
        grid=Grid(np.zeros((80,80),np.int16),.1,(0.,0.,0.))
        pose=(2.,3.,0.)
        identity=(0.,0.,0.)
        def frame(t):
            msg.header.stamp=Time(seconds=t).to_msg()
            emitted.clear()
            publish(node,msg,im,None)
            self.assertTrue(emitted)
            for hit in emitted:
                memory.observe(hit,t)
        frame(1.)
        self.assertIsNone(emitted[0]['point'])
        look=memory.choose(grid,grid,pose,identity,{},1.,lambda g:False)
        self.assertEqual(look['nbv_action'],'relook')
        memory.dispatched(look,1.)
        node.lidar_point=lambda ray,frame,stamp:3.*ray
        for t in (2.,2.2,2.4,2.6,3.2):
            frame(t)
        approach=memory.choose(grid,grid,pose,identity,{},3.2,lambda g:False)
        self.assertEqual(approach['nbv_action'],'approach')
        self.assertGreater(approach['range_m'],3.)
        self.assertFalse(approach['stationary'])
        self.assertGreater(approach['confidence'],.62)

if __name__=='__main__':
    unittest.main()
