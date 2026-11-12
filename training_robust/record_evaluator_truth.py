"""Read-only high-rate evaluator ground truth; never a robot policy input."""
import argparse
import csv
import json
import math
from pathlib import Path
import os
import time


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from tf2_msgs.msg import TFMessage
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain',type=int,required=True)
    parser.add_argument('--native-pid',type=int,required=True)
    parser.add_argument('--native-output',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    context=rclpy.Context();rclpy.init(context=context,domain_id=args.domain)
    node=Node('independent_fine_ground_truth_recorder',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    meta=dict(pid=os.getpid(),native_pid=args.native_pid,domain=args.domain,samples=0,
        file=str(args.output),minimum_sample_period_sim_s=.02,complete=False,
        source='ROS _internal/sim_ground_truth_pose; evaluator only; no publications or commands')
    last=-float('inf');flush=time.monotonic();start=flush;gaps=[]
    with args.output.open('x') as stream:
        writer=csv.writer(stream);writer.writerow(['sim_seconds','world_x','world_y','world_yaw'])
        def receive(message):
            nonlocal last
            for transform in message.transforms:
                if transform.child_frame_id!='turtlebot4':continue
                stamp=transform.header.stamp.sec+transform.header.stamp.nanosec/1e9
                if stamp<last:
                    meta['clock_resets']=meta.get('clock_resets',0)+1;continue
                if stamp-last<.02-1e-9:continue
                p=transform.transform.translation;q=transform.transform.rotation
                if math.isfinite(last):gaps.append(stamp-last)
                writer.writerow([stamp,p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
                last=stamp;meta['samples']+=1
        node.create_subscription(TFMessage,'_internal/sim_ground_truth_pose',receive,qos_profile_sensor_data)
        try:
            while time.monotonic()-start<5500:
                executor.spin_once(timeout_sec=.2)
                if time.monotonic()-flush<1:continue
                stream.flush();flush=time.monotonic()
                try:argv=Path(f'/proc/{args.native_pid}/cmdline').read_bytes().decode().split('\0')
                except OSError:argv=[]
                if not (any(v.endswith('/gazebo_benchmark.py') for v in argv) and args.native_output in argv):
                    meta.update(complete=True,stop_reason='native_process_ended');break
                meta['last_stamp_sim_s']=last if math.isfinite(last) else None
                args.output.with_suffix('.json').write_text(json.dumps(meta,indent=2)+'\n')
        finally:
            meta['max_sample_gap_sim_s']=max(gaps) if gaps else None
            meta['mean_sample_gap_sim_s']=sum(gaps)/len(gaps) if gaps else None
            args.output.with_suffix('.json').write_text(json.dumps(meta,indent=2)+'\n')
            executor.shutdown();node.destroy_node();rclpy.shutdown(context=context)


if __name__=='__main__':main()
