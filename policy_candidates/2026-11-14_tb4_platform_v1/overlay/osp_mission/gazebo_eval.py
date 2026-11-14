from osp_mission.result_io import read_result
"""Isolated Gazebo benchmark with real camera decoder and SLAM; wall watchdog."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, LaserScan, Imu
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from osp_mission.marker_event_journal import MarkerEventJournal
from osp_mission.runtime_metrics import RuntimeMetrics
from osp_mission.native_audit import NativeAudit, yaw
import yaml


def one(scenario,policy,output,domain=84,duration=600.,wall_timeout=5400.,nav2_params='',planner='legacy',camera_width=320,record_video=False,render_backend='software',start_docked=False):
    if render_backend not in ('software','system'):raise ValueError('Invalid rendering backend')
    output=Path(output).resolve(); output.mkdir(parents=True,exist_ok=True)
    # Image transport is configured only on image nodes by their launch files.
    # Enlarging every participant's message buffers exhausts desktop RAM.
    env=dict(os.environ,ROS_DOMAIN_ID=str(domain))
    env['ROS_HOME']=str(output/'ros_home')
    env['ROS_LOG_DIR']=str(output/'ros_logs')
    stream=(output/'launch.log').open('w')
    cmd=['ros2','launch','osp_mission','simulation.launch.py','headless:=true','rviz:=false',
         f'scenario:={Path(scenario).resolve()}',f'policy_file:={Path(policy).resolve()}',
         f'output_dir:={output}',f'duration:={duration}',f'planner:={planner}',f'camera_width:={camera_width}',f'render_backend:={render_backend}',f'start_docked:={str(start_docked).lower()}']
    if nav2_params: cmd.append(f'nav2_params:={Path(nav2_params).resolve()}')
    if record_video:cmd.append('record_video:=true')
    os.environ['ROS_LOG_DIR']=env['ROS_LOG_DIR']
    context=rclpy.Context()
    try:
        rclpy.init(context=context,domain_id=domain)
        node=Node('gazebo_evaluator',context=context)
    except Exception as exc:
        if context.ok(): rclpy.shutdown(context=context)
        stream.close()
        report=dict(reason='evaluator_startup_failure',success=False,error=str(exc),environment='gazebo_harmonic',runtime_metrics=RuntimeMetrics().summary())
        (output/'evaluation.json').write_text(json.dumps(report,indent=2))
        return report
    executor=SingleThreadedExecutor(context=context); executor.add_node(node); client=node.create_client(Trigger,'mission/start')
    status={};detector_status={};metrics=RuntimeMetrics()
    journal=MarkerEventJournal(output/'marker_events.jsonl')
    controller_snapshot={};controller_request=None
    controller_client=node.create_client(GetParameters,'diffdrive_controller/get_parameters')
    controller_keys=['wheel_separation','wheel_radius','wheel_separation_multiplier',
                     'left_wheel_radius_multiplier','right_wheel_radius_multiplier']
    audit=NativeAudit(yaml.safe_load(Path(scenario).read_text()))
    tf=Buffer(node=node);tf_listener=TransformListener(tf,node)
    ground_truth=[];pose_trace=[];marker_observations=[];latest_odom=[];latest_imu=[]
    def on_truth(message):
        for transform in message.transforms:
            if transform.child_frame_id!='turtlebot4':continue
            ground_truth[:]=[transform]
            if started:
                p=transform.transform.translation
                audit.observe(stamp(transform.header.stamp),p.x,p.y,yaw(transform.transform.rotation))
                seconds=stamp(transform.header.stamp)
                if not pose_trace or seconds-pose_trace[-1][0]>=.2:
                    row=[seconds,p.x,p.y,yaw(transform.transform.rotation)]
                    if latest_odom:
                        odom=latest_odom[0];op=odom.pose.pose.position
                        row += [stamp(odom.header.stamp),op.x,op.y,yaw(odom.pose.pose.orientation)]
                    else:row += [None]*4
                    try:
                        mapped=tf.lookup_transform('map','base_link',Time());mp=mapped.transform.translation
                        row += [stamp(mapped.header.stamp),mp.x,mp.y,yaw(mapped.transform.rotation)]
                    except Exception:row += [None]*4
                    row += [stamp(latest_imu[0].header.stamp),latest_imu[0].angular_velocity.z] if latest_imu else [None]*2
                    pose_trace.append(row)
    node.create_subscription(TFMessage,'_internal/sim_ground_truth_pose',on_truth,qos_profile_sensor_data)
    def on_status(message):
        status.update(json.loads(message.data))
        journal.mission(status)
        if status.get('state') in ('saving','done'):metrics.stop()
    node.create_subscription(String,'mission/status',on_status,10)
    def on_detector(message):
        detector_status.update(json.loads(message.data))
        journal.detector(detector_status,status)
    node.create_subscription(String,'mission/detector_status',on_detector,10)
    node.create_subscription(String,'mission/marker_observations',lambda m:marker_observations.append(json.loads(m.data)),10)
    def stamp(value):return value.sec+value.nanosec/1e9
    node.create_subscription(Clock,'clock',lambda m:metrics.add('clock',stamp(m.clock),time.monotonic()),qos_profile_sensor_data)
    node.create_subscription(CameraInfo,'mission/processed_camera',lambda m:metrics.add('camera',stamp(m.header.stamp),time.monotonic()),qos_profile_sensor_data)
    node.create_subscription(LaserScan,'scan',lambda m:metrics.add_scan(stamp(m.header.stamp),time.monotonic(),m.ranges,m.range_min,m.range_max),qos_profile_sensor_data)
    def on_odom(message):
        latest_odom[:]=[message]
        metrics.add('odom',stamp(message.header.stamp),time.monotonic(),
            math.hypot(message.twist.twist.linear.x,message.twist.twist.linear.y),message.twist.twist.angular.z)
    node.create_subscription(Odometry,'odom',on_odom,qos_profile_sensor_data)
    node.create_subscription(Imu,'imu',lambda m:latest_imu.__setitem__(slice(None),[m]),qos_profile_sensor_data)
    node.create_subscription(TwistStamped,'cmd_vel',lambda m:metrics.add('command',stamp(m.header.stamp),time.monotonic(),
        math.hypot(m.twist.linear.x,m.twist.linear.y),m.twist.angular.z),qos_profile_sensor_data)
    request=None; started=False; first=time.monotonic(); report=None; result_file=None
    next_live=first
    def save_audit():
        audit.save(output/'ground_truth_trajectory.csv')
        with (output/'pose_trace.csv').open('w') as target:
            writer=csv.writer(target)
            writer.writerow(['gt_stamp','gt_x','gt_y','gt_yaw','odom_stamp','odom_x','odom_y','odom_yaw',
                             'map_stamp','map_x','map_y','map_yaw','imu_stamp','imu_angular_z'])
            writer.writerows(pose_trace)
        (output/'marker_observations.json').write_text(json.dumps(marker_observations,indent=2))
    process=None
    try:
        process=subprocess.Popen(cmd,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        while time.monotonic()-first<wall_timeout and process.poll() is None:
            executor.spin_once(timeout_sec=.2)
            if not controller_snapshot and controller_client.service_is_ready():
                if controller_request is None:
                    req=GetParameters.Request();req.names=controller_keys
                    controller_request=controller_client.call_async(req)
                elif controller_request.done():
                    controller_snapshot.update({k:v.double_value for k,v in
                        zip(controller_keys,controller_request.result().values)})
            if not started and audit.world_to_map is None and ground_truth:
                truth=ground_truth[0]
                try:
                    # Before mission/start the docked robot is stationary. Allow
                    # the ordinary TF publishing delay when aligning the frames.
                    mapped=tf.lookup_transform('map','base_link',Time())
                    if abs(stamp(mapped.header.stamp)-stamp(truth.header.stamp))>.25:continue
                    wp=truth.transform.translation;mp=mapped.transform.translation
                    audit.calibrate((wp.x,wp.y,yaw(truth.transform.rotation)),
                                    (mp.x,mp.y,yaw(mapped.transform.rotation)))
                except Exception:pass
            if time.monotonic()>=next_live:
                live={'status':status,'detector':detector_status,'controller_parameters':controller_snapshot,'imu_received':bool(latest_imu),
                      'wall_seconds':time.monotonic()-first,'runtime_metrics':metrics.summary()}
                temporary=output/'live_status.tmp'
                temporary.write_text(json.dumps(live,indent=2));temporary.replace(output/'live_status.json')
                metrics.save_trace(output/'runtime_trace.csv')
                save_audit()
                next_live=time.monotonic()+15.
            if not started and audit.world_to_map is not None and latest_imu and client.service_is_ready():
                if request is None: request=client.call_async(Trigger.Request())
                elif request.done():
                    started=request.result().success; request=None
                    if started:metrics.start(time.monotonic())
                    if not started: time.sleep(.5)
            results=sorted(output.glob('*/result.json'))
            if results:
                loaded=read_result(results[-1])
                if loaded is not None:
                    result_file=results[-1];report=loaded;metrics.stop();break
            if not started and time.monotonic()-first>180: break
        if report is None: report={'reason':'startup_failure' if not started else 'wall_watchdog','success':False,'last_status':status}
        report['ground_truth_audit']=audit.evaluate(result_file.parent/'markers.csv' if result_file else None)
        report['success']=bool(report.get('reason')=='all_targets_found' and report.get('elapsed_seconds',float('inf'))<=duration
                               and report.get('collisions',1)==0 and all(report.get('files',{}).values())
                               and report['ground_truth_audit']['all_approached']
                               and report['ground_truth_audit']['correct_ids'] and report['ground_truth_audit']['correct_positions'])
        report.update(environment='gazebo_harmonic',scenario=str(Path(scenario).resolve()),policy=str(Path(policy).resolve()),
                      wall_seconds=time.monotonic()-first,runtime_metrics=metrics.summary(),
                      dds_transport=env.get('FASTDDS_BUILTIN_TRANSPORTS','middleware_default'),detector=detector_status,
                      controller_parameters=controller_snapshot,render_backend=render_backend,start_docked=start_docked,
                      state_estimation='official_gazebo_wheel_encoder_odometry_not_create3_firmware_fusion')
        metrics.save_trace(output/'runtime_trace.csv')
        save_audit()
        (output/'evaluation.json').write_text(json.dumps(report,indent=2))
    finally:
        executor.shutdown(); node.destroy_node(); rclpy.shutdown(context=context)
        try:
            if process is not None: os.killpg(process.pid,signal.SIGINT)
        except ProcessLookupError: pass
        try:
            if process is not None: process.wait(timeout=12)
        except subprocess.TimeoutExpired: pass
        # Gazebo may leave children alive after the launch parent exits: kill the isolated process group.
        try:
            if process is not None: os.killpg(process.pid,signal.SIGKILL)
        except ProcessLookupError: pass
        stream.close()
    if record_video:
        metadata=output/'video/recording.json'
        report['recording']=json.loads(metadata.read_text()) if metadata.exists() else {'video':None,'error':'Recorder did not produce metadata'}
        (output/'evaluation.json').write_text(json.dumps(report,indent=2))
    return report


def main():
    p=argparse.ArgumentParser(); p.add_argument('--scenario',required=True); p.add_argument('--policy',required=True)
    p.add_argument('--output',required=True); p.add_argument('--domain',type=int,default=84)
    p.add_argument('--duration',type=float,default=600); p.add_argument('--wall-timeout',type=float,default=5400)
    p.add_argument('--nav2-params',default='')
    p.add_argument('--record-video',action='store_true')
    p.add_argument('--render-backend',choices=['software','system'],default='software')
    p.add_argument('--start-docked',action='store_true')
    p.add_argument('--planner',choices=['legacy','inspection','directional','surface_endpoint','edge'],default='legacy');p.add_argument('--camera-width',type=int,default=320)
    a=p.parse_args(); print(json.dumps(one(a.scenario,a.policy,a.output,a.domain,a.duration,a.wall_timeout,a.nav2_params,a.planner,a.camera_width,a.record_video,a.render_backend,a.start_docked),indent=2),flush=True)

if __name__=='__main__': main()
