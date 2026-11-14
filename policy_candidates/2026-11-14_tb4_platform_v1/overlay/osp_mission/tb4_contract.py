"""Observe the actual ROS interface before allowing a mission to move."""
from collections import deque
import json
import math
from pathlib import Path
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, Imu, LaserScan
from irobot_create_msgs.msg import HazardDetectionVector
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from osp_mission.tb4_platform import load_profile, contract_errors


class PlatformContract(Node):
    def __init__(self):
        super().__init__('tb4_contract')
        for key,value in dict(output_dir='',base_frame='base_link',odom_frame='odom',scan_topic='scan',
                rgb_topic='oakd/rgb/preview/image_raw',camera_info_topic='oakd/rgb/preview/camera_info').items():
            self.declare_parameter(key,value)
        self.p=lambda key:self.get_parameter(key).value
        self.profile=load_profile(get_package_share_directory('osp_mission'))
        self.profile.update(base_frame=self.p('base_frame'),odom_frame=self.p('odom_frame'))
        self.samples={};self.metadata={};self.last_odom_tf=-math.inf
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        for key,topic,cls in [('odom','odom',Odometry),('scan',self.p('scan_topic'),LaserScan),
                ('rgb',self.p('rgb_topic'),Image),('imu','imu',Imu),('hazards','hazard_detection',HazardDetectionVector),
                ('camera_info',self.p('camera_info_topic'),CameraInfo)]:
            self.create_subscription(cls,topic,lambda msg,k=key:self.receive(k,msg),qos_profile_sensor_data)
        self.create_subscription(TFMessage,'tf',self.on_tf,100)
        self.publisher=self.create_publisher(String,'mission/platform_status',10)
        self.last_errors=None
        self.create_timer(1.,self.report)

    def now(self):return self.get_clock().now().nanoseconds/1e9

    def receive(self,key,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        samples=self.samples.setdefault(key,deque(maxlen=400))
        if samples and stamp<samples[-1]:samples.clear()
        if not samples or stamp>samples[-1]:samples.append(stamp)
        data={'frame':msg.header.frame_id}
        if key=='odom':data['child_frame']=msg.child_frame_id
        if key in ('rgb','camera_info'):data.update(width=msg.width,height=msg.height)
        if key=='camera_info':data['valid_intrinsics']=bool(all(math.isfinite(x) for x in msg.k) and msg.k[0]>0 and msg.k[4]>0)
        if key=='scan':data.update(samples=len(msg.ranges),range_min=msg.range_min,range_max=msg.range_max)
        self.metadata[key]=data

    def on_tf(self,msg):
        for tr in msg.transforms:
            if tr.header.frame_id==self.profile['odom_frame'] and tr.child_frame_id==self.profile['base_frame']:
                self.last_odom_tf=self.now()

    def report(self):
        now=self.now();observed={}
        for key,samples in self.samples.items():
            recent=[s for s in samples if now-5 <= s <= now+.1]
            deltas=[b-a for a,b in zip(recent,recent[1:]) if b>a]
            observed[key]=dict(self.metadata[key],count=len(recent),age_s=round(now-samples[-1],4),
                rate_hz=round(1/statistics.median(deltas),3) if deltas else 0.)
        observed['odom_publishers']=self.count_publishers('odom')
        # Jazzy rclpy exposes timestamps/sequence numbers, not per-message GIDs.
        # Match graph endpoints to the sole odom producer and reject unexpected
        # TF publishers; freshness additionally requires the actual odom edge.
        owners={(e.node_namespace,e.node_name) for e in self.get_publishers_info_by_topic('odom')}
        tf_endpoints=self.get_publishers_info_by_topic('tf')
        observed['odom_tf_authorities']=sum((e.node_namespace,e.node_name) in owners for e in tf_endpoints) if now-self.last_odom_tf<=2.5 else 0
        observed['unexpected_tf_publishers']=[e.node_namespace+'/'+e.node_name for e in tf_endpoints
            if (e.node_namespace,e.node_name) not in owners and e.node_name not in
            ('robot_state_publisher','slam_toolbox','odom_base_tf_bridge')]
        observed['tf_authority_method']='graph endpoint ownership plus observed odom edge, not per-message GID'
        velocity=self.resolve_topic_name(self.profile['velocity_topic'])
        observed['velocity_types']=sorted(dict(self.get_topic_names_and_types()).get(velocity,[]))
        observed['velocity_subscribers']=self.count_subscribers(velocity)
        frames=[observed.get(k,{}).get('frame','') for k in ('rgb','scan','imu')]
        observed['sensor_tf_connected']=all(f and self.tf.can_transform(self.profile['base_frame'],f,Time()) for f in frames)
        errors=contract_errors(observed,self.profile)
        result=dict(profile_id=self.profile['profile_id'],passed=not errors,errors=errors,
            stamp=now,use_sim_time=self.get_parameter('use_sim_time').value,observed=observed,
            hardware_calibrated=False,ground_truth_input=False)
        self.publisher.publish(String(data=json.dumps(result)))
        if errors!=self.last_errors:
            self.get_logger().info('Platform contract: '+('passed' if not errors else '; '.join(errors)))
            self.last_errors=errors
        if self.p('output_dir'):
            path=Path(self.p('output_dir'))/'tb4_platform_observation.json';path.parent.mkdir(parents=True,exist_ok=True)
            temp=path.with_suffix('.tmp');temp.write_text(json.dumps(result,indent=2)+'\n');temp.replace(path)


def main(args=None):
    rclpy.init(args=args);node=PlatformContract()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
