from types import SimpleNamespace
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import TransformStamped
from osp_mission.detector import Detector


def test_size_independent_depth_and_confirmation():
    rclpy.init()
    node=Detector()
    try:
        tr=TransformStamped(); tr.transform.rotation.x=-0.7071067811865476; tr.transform.rotation.w=0.7071067811865476
        identity=TransformStamped(); identity.transform.rotation.w=1.
        node.tf=SimpleNamespace(lookup_transform=lambda target,source,*a,**kw: tr if source=='camera' else identity)
        info=CameraInfo(); info.width=640; info.height=480
        info.k=[500.,0.,320.,0.,500.,240.,0.,0.,1.]; info.d=[0.]*5
        node.on_info(info)
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        captured=[]; node.pub=SimpleNamespace(publish=captured.append)
        frames=[];node.processed_camera=SimpleNamespace(publish=frames.append)
        for index,size in enumerate([80,100,120]):
            img=np.full((480,640),255,dtype=np.uint8)
            img[240-size//2:240+size//2,320-size//2:320+size//2]=cv2.aruco.drawMarker(dictionary,42,size)
            rgb=CvBridge().cv2_to_imgmsg(img,'mono8'); rgb.header.frame_id='camera'; rgb.header.stamp.sec=index+1
            depth=CvBridge().cv2_to_imgmsg(np.full((480,640),2.,dtype=np.float32),'32FC1'); depth.header=rgb.header
            node.on_depth(depth); node.on_rgb(rgb)
        assert len(node.observations[42])==3
        assert len(frames)==3 and frames[-1].header.stamp.sec==3
        node.publish()
        points=[m for m in captured[-1].markers if m.ns=='point']
        assert len(points)==1 and points[0].id==42
        assert abs(points[0].pose.position.y-2.)<.01
        assert abs(points[0].pose.position.x)<.01
        # Later odometry corrections must not relocate a past map landmark.
        correction=TransformStamped();correction.transform.rotation.w=1.
        correction.transform.translation.x=1.2;correction.transform.translation.y=-.8
        node.tf=SimpleNamespace(lookup_transform=lambda *a,**kw:correction)
        node.publish()
        repeated=[m for m in captured[-1].markers if m.ns=='point'][0]
        assert abs(repeated.pose.position.x-points[0].pose.position.x)<1e-9
        assert abs(repeated.pose.position.y-points[0].pose.position.y)<1e-9
        # A floor pattern can decode as Original ArUco; horizontal planes are excluded.
        node.observations.clear()
        node.tf=SimpleNamespace(lookup_transform=lambda *a,**kw:identity)
        rgb.header.stamp.sec=10; depth.header=rgb.header
        node.on_depth(depth); node.on_rgb(rgb)
        assert not node.observations
        # Blank but decoded frames are valid inspection evidence. Invalid
        # calibration or resolution must not emit an inspection heartbeat.
        blank=CvBridge().cv2_to_imgmsg(np.full((480,640),255,dtype=np.uint8),'mono8')
        blank.header=rgb.header;before=len(frames);node.on_rgb(blank)
        assert len(frames)==before+1
        info.width=320;node.on_rgb(blank)
        assert len(frames)==before+1
    finally:
        node.destroy_node(); rclpy.shutdown()
