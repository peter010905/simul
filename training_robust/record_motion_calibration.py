"""Read-only encoder/EKF/IMU/ground-truth motion capture;never a robot policy input."""
import argparse
import csv
import hashlib
import math
from pathlib import Path
import time

from continual_native_curriculum import save


def owner_alive(pid,output):
    try:argv=Path(f'/proc/{pid}/cmdline').read_bytes().decode().split('\0')
    except OSError:return False
    return any(value.endswith('/gazebo_benchmark.py') for value in argv) and str(output) in argv


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from tf2_msgs.msg import TFMessage
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain',type=int,required=True);parser.add_argument('--native-pid',type=int,required=True)
    parser.add_argument('--native-output',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not owner_alive(args.native_pid,args.native_output):raise RuntimeError('Require actual bound native handle')
    context=rclpy.Context();rclpy.init(context=context,domain_id=args.domain)
    node=Node('read_only_motion_calibration_recorder',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    meta=dict(native_pid=args.native_pid,native_output=str(args.native_output),domain=args.domain,
        complete=False,counts={},source='Read-only raw wheel/odom,filtered odom,imu,and independent evaluator ground truth',
        ground_truth_available_to_policy=False,recorder_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    started=flush=time.monotonic();last_truth=-float('inf')
    with args.output.open('x') as stream:
        writer=csv.writer(stream);writer.writerow(['stream','stamp_sim_s','receipt_wall_epoch_s','frame','child_frame',
            'x','y','yaw','linear_x','linear_y','angular_z','variance_linear_x','variance_angular_z'])
        def stamp(header):return header.stamp.sec+header.stamp.nanosec/1e9
        def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        def write(kind,when,frame,child,position,velocity,covariance):
            writer.writerow([kind,when,time.time(),frame,child,*position,*velocity,*covariance])
            meta['counts'][kind]=meta['counts'].get(kind,0)+1
        def odometry(kind,message):
            p=message.pose.pose.position;t=message.twist.twist;c=message.twist.covariance
            write(kind,stamp(message.header),message.header.frame_id,message.child_frame_id,
                [p.x,p.y,yaw(message.pose.pose.orientation)],[t.linear.x,t.linear.y,t.angular.z],[c[0],c[35]])
        def imu(message):
            write('imu',stamp(message.header),message.header.frame_id,'',[None,None,None],
                [None,None,message.angular_velocity.z],[None,message.angular_velocity_covariance[8]])
        def truth(message):
            nonlocal last_truth
            for transform in message.transforms:
                if transform.child_frame_id!='turtlebot4':continue
                when=stamp(transform.header)
                if when-last_truth<.02-1e-9:continue
                p=transform.transform.translation;q=transform.transform.rotation
                write('ground_truth',when,transform.header.frame_id,transform.child_frame_id,
                    [p.x,p.y,yaw(q)],[None,None,None],[None,None]);last_truth=when
        node.create_subscription(Odometry,'wheel/odom',lambda m:odometry('wheel',m),qos_profile_sensor_data)
        node.create_subscription(Odometry,'odom',lambda m:odometry('ekf',m),qos_profile_sensor_data)
        node.create_subscription(Imu,'imu',imu,qos_profile_sensor_data)
        node.create_subscription(TFMessage,'_internal/sim_ground_truth_pose',truth,qos_profile_sensor_data)
        try:
            while time.monotonic()-started<5500:
                executor.spin_once(timeout_sec=.2)
                if time.monotonic()-flush<1.:continue
                stream.flush();flush=time.monotonic()
                if not owner_alive(args.native_pid,args.native_output):
                    meta.update(complete=True,stop_reason='native_process_ended');break
                save(args.output.with_suffix('.json'),meta)
            else:meta['stop_reason']='recorder_wall_limit_without_native_restart'
        finally:
            stream.flush();meta['wall_seconds']=time.monotonic()-started
            save(args.output.with_suffix('.json'),meta)
            executor.shutdown();node.destroy_node();rclpy.shutdown(context=context)


if __name__=='__main__':main()
