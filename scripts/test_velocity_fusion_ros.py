"""Actual ROS encoder->fixed calibration->robot_localization velocity routing smoke test."""
import os
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from ament_index_python.packages import get_package_share_directory,get_package_prefix


def test_real_ekf_fuses_calibrated_body_twist_instead_of_raw_encoder_velocity(tmp_path):
    package=Path(get_package_share_directory('osp_mission'))
    executable=Path(get_package_prefix('osp_mission'))/'lib/osp_mission/calibrated_wheel_twist'
    context=rclpy.Context();rclpy.init(context=context,domain_id=123)
    node=Node('encoder_calibration_fusion_test_peer',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    wheel=node.create_publisher(Odometry,'wheel/odom',10);imu=node.create_publisher(Imu,'imu',10)
    clocks=node.create_publisher(Clock,'clock',10);observed=[]
    node.create_subscription(Odometry,'calibration_test/odom',observed.append,10)
    environment=dict(os.environ,ROS_DOMAIN_ID='123');processes=[]
    with (tmp_path/'fusion.log').open('w') as stream:
        try:
            processes.append(subprocess.Popen([str(executable),'--ros-args','-p','use_sim_time:=true',
                '-p','linear_scale:=0.78'],env=environment,stdout=stream,stderr=stream))
            processes.append(subprocess.Popen(['/opt/ros/jazzy/lib/robot_localization/ekf_node','--ros-args',
                '-r','__node:=state_estimator','--params-file',str(package/'config/sim_ekf.yaml'),
                '-r','odometry/filtered:=calibration_test/odom'],env=environment,stdout=stream,stderr=stream))
            started=time.monotonic();index=0
            while time.monotonic()-started<12.:
                assert all(process.poll() is None for process in processes)
                stamp=10.+index*.05;index+=1
                clock=Clock();clock.clock.sec=int(stamp);clock.clock.nanosec=int(round((stamp-int(stamp))*1e9));clocks.publish(clock)
                raw=Odometry();raw.header.stamp=clock.clock;raw.header.frame_id='odom';raw.child_frame_id='base_link'
                raw.pose.pose.orientation.w=1.;raw.twist.twist.linear.x=.4
                raw.twist.covariance[0]=raw.twist.covariance[7]=.001;raw.twist.covariance[35]=.01;wheel.publish(raw)
                angular=Imu();angular.header.stamp=clock.clock;angular.header.frame_id='base_link'
                angular.angular_velocity_covariance[8]=.01;imu.publish(angular)
                executor.spin_once(timeout_sec=.01);time.sleep(.01)
                if len(observed)>20 and observed[-1].pose.pose.position.x>.5:break
            assert len(observed)>20,(tmp_path/'fusion.log').read_text()
            assert abs(observed[-1].twist.twist.linear.x-.312)<.015
            assert observed[-1].pose.pose.position.x>.5
        finally:
            for process in processes:
                if process.poll() is None:process.terminate()
            for process in processes:
                try:process.wait(timeout=5.)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5.)
            executor.shutdown();node.destroy_node();rclpy.shutdown(context=context)
