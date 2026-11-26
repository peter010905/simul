from types import SimpleNamespace
import math
import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from osp_mission.detector import Detector


def test_laser_plane_uses_laser_time_and_camera_time_in_the_same_odom_frame():
    rclpy.init();node=Detector()
    try:
        scan=LaserScan();scan.header.frame_id='laser';scan.header.stamp.sec=30
        scan.angle_min=-.12;scan.angle_increment=.01;scan.range_min=.05;scan.range_max=10.
        scan.ranges=[2./math.cos(scan.angle_min+i*scan.angle_increment) for i in range(25)]
        node.on_scan(scan)
        transform=TransformStamped()
        transform.transform.translation.x=.2
        transform.transform.rotation.y=math.sqrt(.5);transform.transform.rotation.w=math.sqrt(.5)
        calls=[]
        def lookup(target,target_time,source,source_time,fixed,**kw):
            calls.append((target,target_time.nanoseconds,source,source_time.nanoseconds,fixed,kw))
            return transform
        node.tf=SimpleNamespace(lookup_transform_full=lookup)
        from builtin_interfaces.msg import Time as Stamp
        point=node.lidar_point(np.array([0.,0.,1.]),'camera',Time.from_msg(Stamp(sec=30,nanosec=200_000_000)))
        # The wall was 2m from the older scan origin; the camera moved 0.2m
        # toward it. Reusing a rigid same-time transform incorrectly gives 2m.
        np.testing.assert_allclose(point,[0.,0.,1.8],atol=1e-6)
        assert calls[0][:5]==('laser',30_000_000_000,'camera',30_200_000_000,'odom')
        assert calls[0][5]['timeout'].nanoseconds==0
    finally:
        node.destroy_node();rclpy.shutdown()
