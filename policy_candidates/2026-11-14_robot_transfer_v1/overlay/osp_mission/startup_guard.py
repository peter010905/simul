"""Start SLAM through observed lifecycle states, then release the Nav2 launch."""
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from rclpy.time import Time
from lifecycle_msgs.srv import GetState,ChangeState
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer,TransformListener

from osp_mission.startup_state import StartupTransitions


class StartupGuard(Node):
    def __init__(self,**kwargs):
        super().__init__('slam_startup_guard',**kwargs)
        for key,value in dict(output_dir='/tmp/osp-startup',startup_timeout_wall_s=120.,base_frame='base_link',scan_topic='scan').items():
            self.declare_parameter(key,value)
        self.started=time.monotonic();self.finished=False;self.success=False
        self.events=[];self.transitions=StartupTransitions();self.query=None;self.change=None
        self.requested_at=0.;self.last_state=None;self.map_ready=False;self.scan_stamp=None
        self.get_state=self.create_client(GetState,'slam_toolbox/get_state')
        self.change_state=self.create_client(ChangeState,'slam_toolbox/change_state')
        self.tf=Buffer(node=self);self.listener=TransformListener(self.tf,self)
        self.create_subscription(OccupancyGrid,'map',self.on_map,QoSProfile(depth=1,
            reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan,self.get_parameter('scan_topic').value,self.on_scan,qos_profile_sensor_data)
        self.timer=self.create_timer(.25,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_map(self,message):
        self.map_ready=bool(message.info.width>0 and message.info.height>0 and len(message.data)>0)

    def on_scan(self,message):
        self.scan_stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9

    def event(self,kind,**data):
        self.events.append(dict(event=kind,elapsed_wall_s=time.monotonic()-self.started,**data))
        self.get_logger().info(f'Startup {kind}: {data}')

    def finish(self,success,reason):
        self.finished=True;self.success=success
        self.event('finished',success=success,reason=reason)
        output=Path(self.get_parameter('output_dir').value);output.mkdir(parents=True,exist_ok=True)
        report=dict(success=success,reason=reason,finished_wall_epoch_s=time.time(),elapsed_wall_s=time.monotonic()-self.started,
            observed_slam_state=self.last_state,transitions=self.transitions.attempts,events=self.events,
            map_ready=self.map_ready,nav2_released_after_readiness=success)
        report['tf_ready'] = {parent: self.tf.can_transform(parent,
            self.get_parameter('base_frame').value,Time()) for parent in ('map','odom')}
        report['scan_stamp'] = self.scan_stamp
        report['clock_stamp'] = self.get_clock().now().nanoseconds/1e9
        temporary=output/'startup_guard.json.tmp';temporary.write_text(json.dumps(report,indent=2)+'\n')
        temporary.replace(output/'startup_guard.json')

    def ready(self):
        now=self.get_clock().now().nanoseconds/1e9
        return bool(self.last_state==3 and self.map_ready and self.scan_stamp is not None
            and -.1<=now-self.scan_stamp<=2. and self.tf.can_transform(
                'map',self.get_parameter('base_frame').value,Time()))

    def tick(self):
        if self.finished:return
        wall=time.monotonic()
        if wall-self.started>self.get_parameter('startup_timeout_wall_s').value:
            self.finish(False,'startup_readiness_timeout');return
        if self.change is not None:
            if self.change.done():
                try:self.event('transition_response',success=self.change.result().success)
                except Exception as exc:self.event('transition_response_error',error=str(exc))
                self.change=None;self.last_state=None
            elif wall-self.requested_at>15.:
                self.change.cancel();self.change_state.remove_pending_request(self.change)
                self.change=None;self.last_state=None
                self.event('transition_response_missing_requery_state')
            return
        if self.query is not None:
            if not self.query.done():
                if wall-self.requested_at>5.:
                    self.query.cancel();self.get_state.remove_pending_request(self.query);self.query=None
                    self.event('state_observation_timeout')
                return
            try:state=int(self.query.result().current_state.id)
            except Exception as exc:
                self.event('state_observation_error',error=str(exc));self.query=None;return
            self.query=None
            if state!=self.last_state:self.event('state_observed',state=state)
            self.last_state=state
            if self.ready():self.finish(True,'slam_active_map_tf_scan_ready');return
            if self.transitions.exhausted(state):self.finish(False,'transition_attempt_limit');return
            if self.change_state.service_is_ready():
                transition=self.transitions.next(state)
                if transition is not None:
                    request=ChangeState.Request();request.transition.id=transition
                    self.change=self.change_state.call_async(request);self.requested_at=wall
                    self.event('transition_requested',from_state=state,transition=transition)
                    return
        if self.query is None and self.get_state.service_is_ready():
            self.query=self.get_state.call_async(GetState.Request());self.requested_at=wall


def main(args=None):
    rclpy.init(args=args);node=StartupGuard()
    try:
        while rclpy.ok() and not node.finished:rclpy.spin_once(node,timeout_sec=.2)
        return 0 if node.success else 1
    finally:
        node.destroy_node();rclpy.shutdown()
