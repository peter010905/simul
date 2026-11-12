"""ROS service/TF readiness smoke test with a controlled SLAM lifecycle peer."""
import json
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from lifecycle_msgs.srv import GetState,ChangeState
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from osp_mission.startup_guard import StartupGuard


@pytest.mark.parametrize('initial',[1,2,3])
def test_real_ros_lifecycle_polling_releases_only_after_map_tf_and_scan(tmp_path,initial):
    context=rclpy.Context();rclpy.init(context=context,domain_id=121)
    peer=Node('controlled_slam_startup_peer',context=context);state=[initial];requests=[]
    def get(req,res):res.current_state.id=state[0];return res
    def change(req,res):
        requests.append(req.transition.id)
        if state[0]==1 and req.transition.id==1:state[0]=2;res.success=True
        elif state[0]==2 and req.transition.id==3:state[0]=3;res.success=True
        return res
    peer.create_service(GetState,'slam_toolbox/get_state',get)
    peer.create_service(ChangeState,'slam_toolbox/change_state',change)
    maps=peer.create_publisher(OccupancyGrid,'map',QoSProfile(depth=1,
        reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    scans=peer.create_publisher(LaserScan,'scan',qos_profile_sensor_data)
    clocks=peer.create_publisher(Clock,'clock',10);tf=TransformBroadcaster(peer)
    ready=[False]
    def publish():
        clock=Clock();clock.clock.sec=10;clocks.publish(clock)
        if state[0]!=3 or not ready[0]:return
        m=OccupancyGrid();m.header.frame_id='map';m.info.width=m.info.height=2;m.info.resolution=.1;m.data=[0]*4;maps.publish(m)
        scan=LaserScan();scan.header.stamp.sec=10;scans.publish(scan)
        tr=TransformStamped();tr.header.stamp.sec=10;tr.header.frame_id='map';tr.child_frame_id='base_link';tr.transform.rotation.w=1.;tf.sendTransform(tr)
    timer=peer.create_timer(.05,publish)
    guard=StartupGuard(context=context,parameter_overrides=[Parameter('use_sim_time',value=True),
        Parameter('output_dir',value=str(tmp_path)),Parameter('startup_timeout_wall_s',value=20.)])
    executor=SingleThreadedExecutor(context=context);executor.add_node(peer);executor.add_node(guard)
    start=time.monotonic()
    try:
        while time.monotonic()-start<1.:executor.spin_once(timeout_sec=.05)
        assert not guard.finished  # Lifecycle alone cannot release navigation.
        ready[0]=True
        while not guard.finished and time.monotonic()-start<22.:executor.spin_once(timeout_sec=.05)
        assert guard.finished and guard.success
        assert requests=={1:[1,3],2:[3],3:[]}[initial]
        result=json.loads((tmp_path/'startup_guard.json').read_text())
        assert result['success'] and result['observed_slam_state']==3 and result['map_ready']
    finally:
        executor.shutdown();guard.destroy_node();peer.destroy_node();rclpy.shutdown(context=context)
