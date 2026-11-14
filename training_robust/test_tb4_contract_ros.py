"""Local ROS transport integration test; no Gazebo or velocity publisher."""
import os
import signal
import time
from pathlib import Path
import json
import subprocess


def test_live_sensor_contract_then_sensor_loss(tmp_path):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, CameraInfo, Imu, LaserScan
    from irobot_create_msgs.msg import HazardDetectionVector
    from geometry_msgs.msg import TransformStamped, TwistStamped
    from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
    from std_msgs.msg import String
    # Separate localhost-only domain: the test never publishes velocity.
    domain=213;env=dict(os.environ,ROS_DOMAIN_ID=str(domain),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    context=rclpy.Context();rclpy.init(context=context,domain_id=domain)
    node=Node('tb4_test_sensor_source',context=context);executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    publishers={key:node.create_publisher(cls,topic,qos_profile_sensor_data) for key,topic,cls in [
        ('odom','odom',Odometry),('scan','scan',LaserScan),('rgb','oakd/rgb/preview/image_raw',Image),
        ('info','oakd/rgb/preview/camera_info',CameraInfo),('imu','imu',Imu),('hazards','hazard_detection',HazardDetectionVector)]}
    node.create_subscription(TwistStamped,'cmd_vel',lambda _:None,10)
    states=[];node.create_subscription(String,'mission/platform_status',lambda m:states.append(json.loads(m.data)),10)
    broadcaster=TransformBroadcaster(node);static=StaticTransformBroadcaster(node)
    transforms=[]
    for frame in ['camera_optical','rplidar_link','imu_link']:
        tr=TransformStamped();tr.header.frame_id='base_link';tr.child_frame_id=frame;tr.transform.rotation.w=1.;transforms.append(tr)
    static.sendTransform(transforms)
    next_time={key:0. for key in publishers};rates=dict(odom=20.,scan=5.5,rgb=30.,info=30.,imu=100.,hazards=20.)
    log=(tmp_path/'contract.log').open('w')
    process=subprocess.Popen(['ros2','run','osp_mission','tb4_contract','--ros-args','-p','use_sim_time:=false','-p','output_dir:='+str(tmp_path)],env=env,stdout=log,stderr=log,start_new_session=True)
    deadline=time.monotonic()+18
    try:
        while time.monotonic()<deadline and not any(s['passed'] for s in states):
            now=time.monotonic();stamp=node.get_clock().now().to_msg()
            for key,publisher in publishers.items():
                if now<next_time[key]:continue
                next_time[key]=now+1/rates[key]
                if key=='odom':
                    msg=Odometry();msg.header.frame_id='odom';msg.child_frame_id='base_link';msg.pose.pose.orientation.w=1.
                    tr=TransformStamped();tr.header.frame_id='odom';tr.header.stamp=stamp;tr.child_frame_id='base_link';tr.transform.rotation.w=1.;broadcaster.sendTransform(tr)
                elif key=='scan':
                    msg=LaserScan();msg.header.frame_id='rplidar_link';msg.range_min=.15;msg.range_max=12.;msg.ranges=[1.]*1440
                elif key=='rgb':
                    msg=Image();msg.header.frame_id='camera_optical';msg.width=msg.height=250;msg.encoding='rgb8';msg.step=750;msg.data=bytes(250*750)
                elif key=='info':
                    msg=CameraInfo();msg.header.frame_id='camera_optical';msg.width=msg.height=250;msg.k=[250.,0.,125.,0.,250.,125.,0.,0.,1.]
                elif key=='imu':msg=Imu();msg.header.frame_id='imu_link'
                else:msg=HazardDetectionVector();msg.header.frame_id='base_link'
                msg.header.stamp=stamp;publisher.publish(msg)
            executor.spin_once(timeout_sec=.001)
        assert any(s['passed'] for s in states),(states[-1:] , (tmp_path/'contract.log').read_text())
        states.clear();deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.1)
        assert states and not states[-1]['passed']
        assert any('stale' in error for error in states[-1]['errors'])
    finally:
        os.killpg(process.pid,signal.SIGINT) if process.poll() is None else None
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        log.close();executor.shutdown();node.destroy_node();context.shutdown()
