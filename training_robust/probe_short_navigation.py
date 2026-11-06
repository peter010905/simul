"""Native controller diagnostic, deliberately excluded from mission success rates.

After Nav2 activation, request one 0.2 m forward / 0.4 m lateral displacement
with unchanged final heading in an open area. No mission/start call is made.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
from osp_mission.drive_readiness import DriveReadiness
from osp_mission.lifecycle_readiness import Nav2Readiness
import yaml


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True)
    parser.add_argument('--nav2-params',required=True)
    parser.add_argument('--domain',type=int,default=93)
    parser.add_argument('--dx',type=float,default=.2)
    parser.add_argument('--dy',type=float,default=.4)
    parser.add_argument('--yaw-offset',type=float,default=0.)
    args=parser.parse_args()
    root=Path(args.output).resolve();root.mkdir(parents=True,exist_ok=False)
    scenario=root/'scenario.yaml'
    scenario.write_text(yaml.safe_dump(dict(seed=1,randomize=False,start=[10.,5.5,0.],obstacles=[],targets=[
        dict(id=7,x=3.,y=.15,yaw=math.pi/2,size=.2),
        dict(id=43,x=10.,y=11.18,yaw=-math.pi/2,size=.2),
        dict(id=81,x=19.98,y=5.,yaw=math.pi,size=.2)])))
    env=dict(os.environ,ROS_DOMAIN_ID=str(args.domain),ROS_LOG_DIR=str(root/'ros_logs'),ROS_HOME=str(root/'ros_home'))
    os.environ['ROS_LOG_DIR']=env['ROS_LOG_DIR']
    rclpy.init(domain_id=args.domain)
    node=Node('short_navigation_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer(node=node);listener=TransformListener(buffer,node)
    nav=ActionClient(node,NavigateToPose,'navigate_to_pose')
    nav_state=Nav2Readiness(node);drive_state=DriveReadiness(node)
    wheel_stamp=[None];trace=[];report={};handle=None
    wheel_sub=node.create_subscription(Odometry,'wheel/odom',lambda m:wheel_stamp.__setitem__(0,
        m.header.stamp.sec+m.header.stamp.nanosec/1e9),qos_profile_sensor_data)
    def seconds():return node.get_clock().now().nanoseconds/1e9
    def pose():
        t=buffer.lookup_transform('map','base_link',Time()).transform;q=t.rotation
        return (t.translation.x,t.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    command=['ros2','launch','osp_mission','simulation.launch.py','headless:=true','rviz:=false',
             'record_video:=false','render_backend:=software','start_docked:=false',
             'scenario:='+str(scenario),'nav2_params:='+str(Path(args.nav2_params).resolve()),
             'output_dir:='+str(root)]
    log=(root/'launch.log').open('w')
    process=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        start=time.monotonic();ready=False
        while time.monotonic()-start<180 and process.poll() is None:
            rclpy.spin_once(node,timeout_sec=.1)
            nav_state.poll();drive_state.poll()
            fresh_wheel=wheel_stamp[0] is not None and abs(seconds()-wheel_stamp[0])<.75
            if nav_state.ready() and drive_state.ready() and fresh_wheel and nav.server_is_ready():
                try:initial=pose();ready=True;break
                except Exception:pass
        if not ready:raise RuntimeError('Nav2 and physical drive startup did not complete')
        x,y,a=initial;c,s=math.cos(a),math.sin(a)
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=node.get_clock().now().to_msg()
        goal.pose.pose.position.x=x+args.dx*c-args.dy*s;goal.pose.pose.position.y=y+args.dx*s+args.dy*c
        goal_yaw=a+args.yaw_offset
        goal.pose.pose.orientation.z=math.sin(goal_yaw/2);goal.pose.pose.orientation.w=math.cos(goal_yaw/2)
        started=seconds();wall_started=time.monotonic();future=nav.send_goal_async(goal);result=None
        while time.monotonic()-wall_started<240 and seconds()-started<60 and process.poll() is None:
            rclpy.spin_once(node,timeout_sec=.05)
            if handle is None and future.done():
                handle=future.result()
                if not handle.accepted:raise RuntimeError('Active Nav2 rejected diagnostic goal')
                result=handle.get_result_async()
            if not trace or seconds()-trace[-1][0]>=.2:
                try:trace.append((seconds(),*pose()))
                except Exception:pass
            if result is not None and result.done():break
        status=result.result().status if result is not None and result.done() else None
        report=dict(success=status==4,status=status,elapsed_seconds=seconds()-started,
                    initial=initial,goal=[goal.pose.pose.position.x,goal.pose.pose.position.y,goal_yaw],
                    final=pose(),trace=trace,wall_seconds=time.monotonic()-wall_started,
                    nav2_params=str(Path(args.nav2_params).resolve()),
                    nav2_params_sha256=hashlib.sha256(Path(args.nav2_params).read_bytes()).hexdigest(),
                    drive_controllers_at_start=drive_state.states,
                    scope='Controller diagnostic only; not an autonomous mission or robustness success')
    except Exception as exc:
        report=dict(success=False,error=str(exc),trace=trace)
    finally:
        if handle is not None and handle.accepted:handle.cancel_goal_async()
        try:os.killpg(process.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:process.wait(timeout=12)
        except subprocess.TimeoutExpired:pass
        try:os.killpg(process.pid,signal.SIGKILL)
        except ProcessLookupError:pass
        node.destroy_node();rclpy.shutdown();log.close()
    (root/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='trace'},indent=2))


if __name__=='__main__':main()
