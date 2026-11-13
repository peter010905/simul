"""Replay recorded wheel/IMU data through real isolated EKF nodes,without Gazebo.

Only wheel twist,IMU angular rate and clock go into the ROS components.
Ground truth is read for scoring after replay; this is not a native mission.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np

from continual_native_curriculum import digest,save


def load_inputs(path,start,stop):
    records={}
    for row in csv.DictReader(path.open()):
        kind=row['stream'];stamp=float(row['stamp_sim_s'])
        if kind in ('wheel','imu') and start<=stamp<=stop:
            records[(stamp,kind)]=row
    return sorted(records.items())


def replay(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    args.output.mkdir(parents=True,exist_ok=False)
    inputs=load_inputs(args.input,args.start,args.stop)
    if not inputs:raise ValueError('No wheel/IMU samples in requested segment')
    context=rclpy.Context();rclpy.init(context=context,domain_id=args.domain)
    node=Node('encoder_spike_component_replay',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    clock=node.create_publisher(Clock,'/clock',10)
    outputs={key:[] for key in ('baseline','guard')};statuses={}
    wheels={};imus={};processes=[];streams=[]
    def record(key,message):
        p=message.pose.pose.position;q=message.pose.pose.orientation;t=message.header.stamp
        outputs[key].append(dict(stamp=t.sec+t.nanosec/1e9,x=p.x,y=p.y,
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
            speed=message.twist.twist.linear.x))
    try:
        for key,maximum in [('baseline',0.),('guard',args.guard_maximum)]:
            wheels[key]=node.create_publisher(Odometry,f'/{key}/wheel/odom',qos_profile_sensor_data)
            imus[key]=node.create_publisher(Imu,f'/{key}/imu',qos_profile_sensor_data)
            node.create_subscription(Odometry,f'/{key}/odom',lambda m,k=key:record(k,m),qos_profile_sensor_data)
            node.create_subscription(String,f'/{key}/mission/wheel_twist_status',
                lambda m,k=key:statuses.__setitem__(k,json.loads(m.data)),10)
            common=['--ros-args','-r','__ns:=/'+key,'-p','use_sim_time:=true']
            commands=[
                [str(args.workspace/'install/osp_mission/lib/osp_mission/calibrated_wheel_twist'),*common,
                    '-p',f'linear_scale:={.78 if key=="baseline" else args.guard_scale}',
                    '-p',f'maximum_calibrated_linear_speed:={maximum}'],
                ['/opt/ros/jazzy/lib/robot_localization/ekf_node',*common,
                    '--params-file',str(args.workspace/'src/osp_mission/config/sim_ekf.yaml'),
                    '-r','odometry/filtered:=odom','-r','/tf:=tf','-r','/tf_static:=tf_static']]
            if args.guard_mode:
                commands[0]+=['-p','velocity_source:='+('reported' if key=='baseline' else args.guard_mode)]
            for index,command in enumerate(commands):
                stream=(args.output/f'{key}_{index}.log').open('w');streams.append(stream)
                processes.append(subprocess.Popen(command,stdout=stream,stderr=stream,start_new_session=True,
                    env=dict(os.environ,ROS_DOMAIN_ID=str(args.domain))))
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            ready_clock=Clock();ready_clock.clock.nanosec=1000000
            clock.publish(ready_clock);executor.spin_once(timeout_sec=.02)
            if all(pub.get_subscription_count() for pub in [*wheels.values(),*imus.values()]):break
        else:raise RuntimeError('Isolated EKF components failed discovery')
        position=0;began=time.monotonic();duration=args.stop-args.start+1.
        for tick in range(1,int(round(duration*100))+1):
            relative=tick*.01;nanoseconds=round(relative*1e9)
            when=Clock();when.clock.sec=nanoseconds//1000000000;when.clock.nanosec=nanoseconds%1000000000
            clock.publish(when)
            while position<len(inputs) and inputs[position][0][0]-args.start+1<=relative+1e-9:
                (stamp,kind),row=inputs[position];position+=1
                ns=round((stamp-args.start+1)*1e9)
                if kind=='wheel':
                    m=Odometry();m.header.frame_id='odom';m.child_frame_id='base_link'
                    m.pose.pose.position.x=float(row['x']);m.pose.pose.position.y=float(row['y'])
                    yaw=float(row['yaw']);m.pose.pose.orientation.z=math.sin(yaw/2);m.pose.pose.orientation.w=math.cos(yaw/2)
                    m.twist.twist.linear.x=float(row['linear_x']);m.twist.twist.linear.y=float(row['linear_y'])
                    m.twist.twist.angular.z=float(row['angular_z'])
                    m.twist.covariance[0]=float(row['variance_linear_x']);m.twist.covariance[7]=.001
                    m.twist.covariance[35]=float(row['variance_angular_z']);publishers=wheels
                else:
                    m=Imu();m.header.frame_id='base_link';m.orientation_covariance[0]=-1.
                    m.angular_velocity.z=float(row['angular_z'])
                    m.angular_velocity_covariance[8]=float(row['variance_angular_z']);publishers=imus
                m.header.stamp.sec=ns//1000000000;m.header.stamp.nanosec=ns%1000000000
                for publisher in publishers.values():publisher.publish(m)
            executor.spin_once(timeout_sec=.001)
            remaining=began+relative/args.rate-time.monotonic()
            if remaining>0:time.sleep(remaining)
        flush=time.monotonic()+.5
        while time.monotonic()<flush:executor.spin_once(timeout_sec=.02)
        if any(not values for values in outputs.values()):raise RuntimeError('Both actual EKF output streams required')
        for key,values in outputs.items():save(args.output/f'{key}_poses.json',values)
        save(args.output/'statuses.json',statuses)
    finally:
        for process in processes:
            if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        for process in processes:
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=2)
        for stream in streams:stream.close()
        executor.shutdown();node.destroy_node();rclpy.shutdown(context=context)
    # Independent evaluator only,after ROS replay has finished.
    truth=[]
    for row in csv.DictReader(args.input.open()):
        if row['stream']=='ground_truth':
            stamp=float(row['stamp_sim_s'])
            if args.start<=stamp<=args.stop:truth.append([stamp-args.start+1,float(row['x']),float(row['y']),float(row['yaw'])])
    truth=np.asarray(truth);truth=truth[np.argsort(truth[:,0],kind='stable')]
    scores={}
    for key,values in outputs.items():
        first=next(value for value in values if value['stamp']>=truth[0,0])
        tx=np.interp(first['stamp'],truth[:,0],truth[:,1]);ty=np.interp(first['stamp'],truth[:,0],truth[:,2])
        yaw=np.interp(first['stamp'],truth[:,0],np.unwrap(truth[:,3]))-first['yaw']
        R=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
        errors=[]
        for value in values:
            if not first['stamp']<=value['stamp']<=truth[-1,0]:continue
            predicted=R@(np.array([value['x'],value['y']])-[first['x'],first['y']])+[tx,ty]
            actual=[np.interp(value['stamp'],truth[:,0],truth[:,i]) for i in (1,2)]
            errors.append(float(np.linalg.norm(predicted-actual)))
        scores[key]=dict(samples=len(errors),max_position_error_m=max(errors),last_position_error_m=errors[-1],
            maximum_reported_forward_speed_m_s=max(abs(v['speed']) for v in values),status=statuses.get(key))
    report=dict(evidence_type='recorded_encoder_IMU_replay_through_actual_EKF_components_not_Gazebo_mission',
        input=str(args.input),original_segment_sim_s=[args.start,args.stop],input_samples=len(inputs),
        replay_rate=args.rate,domain=args.domain,guard_mode=args.guard_mode,guard_linear_scale=args.guard_scale,
        guard_maximum_calibrated_speed=args.guard_maximum,scores=scores,
        policy_inputs=['wheel_twist','IMU_angular_velocity','clock'],ground_truth_sent_to_components=False,
        limits=['Recorded subset,initial pose aligned once for scoring;not a new native trajectory or completion result',
            'IMU angular rate is normalized into the base_link frame;no absolute orientation or world position is fused',
            'Replay scheduling differs from live Gazebo;does not establish the cause of every historical EKF jump'],
        source_sha256={str(args.input):digest(args.input),str(Path(__file__)):digest(Path(__file__)),
            'runtime_helper':digest(args.workspace/'src/osp_mission/osp_mission/calibrated_wheel_twist.py'),
            'runtime_EKF_parameters':digest(args.workspace/'src/osp_mission/config/sim_ekf.yaml')})
    save(args.output/'summary.json',report);print(json.dumps(scores))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('input','workspace','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--start',type=float,default=350.);parser.add_argument('--stop',type=float,default=365.)
    parser.add_argument('--domain',type=int,default=93);parser.add_argument('--rate',type=float,default=2.)
    parser.add_argument('--guard-mode',choices=['reported','pose_delta'])
    parser.add_argument('--guard-scale',type=float,default=.78)
    parser.add_argument('--guard-maximum',type=float,default=.75)
    args=parser.parse_args()
    for name in ('input','workspace','output'):setattr(args,name,getattr(args,name).resolve())
    replay(args)
