"""Synthetic ROS camera and delayed TF; never connects to a robot or drives."""
import argparse
import json
import math
import os
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from osp_mission.detector import Detector


class SyntheticCamera(Node):
    def __init__(self,delay):
        super().__init__('synthetic_delayed_camera')
        self.delay=delay;self.bridge=CvBridge()
        self.rgb=self.create_publisher(Image,'oakd/rgb/preview/image_raw',qos_profile_sensor_data)
        self.depth=self.create_publisher(Image,'oakd/rgb/preview/depth',qos_profile_sensor_data)
        self.info=self.create_publisher(CameraInfo,'oakd/rgb/preview/camera_info',qos_profile_sensor_data)
        self.dynamic=TransformBroadcaster(self);self.fixed=StaticTransformBroadcaster(self)
        transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='odom'
        transform.transform.rotation.w=1.;transform.header.stamp=self.get_clock().now().to_msg()
        self.fixed.sendTransform(transform)
        image=np.full((480,480),255,dtype=np.uint8)
        image[190:290,190:290]=cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL),42,100)
        self.image=self.bridge.cv2_to_imgmsg(image,'mono8')
        self.distance=self.bridge.cv2_to_imgmsg(np.full((480,480),2.,dtype=np.float32),'32FC1')
        self.calibration=CameraInfo();self.calibration.width=self.calibration.height=480
        self.calibration.k=[400.,0.,240.,0.,400.,240.,0.,0.,1.];self.calibration.d=[0.]*5
        self.create_timer(.1,self.expose);self.create_timer(.01,self.transform)

    def expose(self):
        stamp=self.get_clock().now().to_msg()
        for message in (self.image,self.distance,self.calibration):
            message.header.stamp=stamp;message.header.frame_id='camera'
        self.info.publish(self.calibration);self.depth.publish(self.distance);self.rgb.publish(self.image)

    def transform(self):
        transform=TransformStamped();transform.header.frame_id='odom';transform.child_frame_id='camera'
        transform.header.stamp=Time(nanoseconds=self.get_clock().now().nanoseconds-int(self.delay*1e9)).to_msg()
        transform.transform.rotation.x=-math.sqrt(.5);transform.transform.rotation.w=math.sqrt(.5)
        transform.transform.translation.z=.45
        self.dynamic.sendTransform(transform)


def probe(delay,duration):
    assert os.environ.get('ROS_DOMAIN_ID')=='182' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    rclpy.init();source=SyntheticCamera(delay);detector=Detector()
    producer=SingleThreadedExecutor();producer.add_node(source)
    thread=threading.Thread(target=producer.spin,daemon=True);thread.start()
    executor=SingleThreadedExecutor();executor.add_node(detector)
    frames=[]
    detector.processed_camera=type('Sink',(),{'publish':lambda self,m:frames.append(m.header.stamp.sec+m.header.stamp.nanosec/1e9)})()
    started=time.monotonic()
    try:
        while time.monotonic()-started<duration:executor.spin_once(timeout_sec=.05)
        from osp_mission.landmark_identity import resolved_ids
        ids,_=resolved_ids(detector.observations,3,separate=detector.separate_markers)
        return dict(delay_s=delay,duration_wall_s=time.monotonic()-started,registered_ids=sorted(ids),
            accepted=detector.diagnostics['accepted'],frames_processed=len(frames),diagnostics=detector.diagnostics,
            queue=getattr(getattr(detector,'exposure_queue',None),'counts',None),
            timing=detector.pipeline_timing.report() if hasattr(detector,'pipeline_timing') else None,
            scope='Synthetic ArUco/depth/TF ROS integration, not Gazebo navigation or physical hardware')
    finally:
        executor.shutdown();producer.shutdown();thread.join(timeout=2)
        detector.destroy_node();source.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('output',type=Path)
    parser.add_argument('--delay',type=float,default=.12);parser.add_argument('--duration',type=float,default=4.)
    args=parser.parse_args();result=probe(args.delay,args.duration)
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
