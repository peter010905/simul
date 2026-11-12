"""Read-only native cliff-sensor and hazard probe. Does not issue robot commands."""
import argparse
import collections
import json
import math
from pathlib import Path
import time


def main():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain',type=int,required=True)
    parser.add_argument('--seconds',type=float,default=8)
    parser.add_argument('--output',type=Path)
    args = parser.parse_args()
    context = rclpy.Context()
    rclpy.init(context=context,domain_id=args.domain)
    node = Node('spatial_floor_readonly_probe',context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    samples = collections.Counter()
    hazards = collections.Counter()
    streams = {}
    def on_hazard(message):
        samples['hazard_frames'] += 1
        samples['hazard_nonempty_frames'] += bool(message.detections)
        for item in message.detections:
            hazards[f'{item.type}:{item.header.frame_id}'] += 1
    node.create_subscription(HazardDetectionVector,'hazard_detection',on_hazard,qos_profile_sensor_data)
    for name in ('cliff_front_left','cliff_front_right','cliff_side_left','cliff_side_right'):
        def receive(message,key=name):
            samples[key] += 1
            info = streams.setdefault(key,dict(values=0,finite=0,infinite=0,last_ranges=[]))
            info['values'] += len(message.ranges)
            info['finite'] += sum(math.isfinite(r) for r in message.ranges)
            info['infinite'] += sum(math.isinf(r) for r in message.ranges)
            info['last_ranges'] = [r if math.isfinite(r) else str(r) for r in message.ranges]
            info['range_min'],info['range_max'] = message.range_min,message.range_max
        node.create_subscription(LaserScan,'_internal/'+name+'/scan',receive,qos_profile_sensor_data)
    deadline = time.monotonic()+args.seconds
    try:
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.2)
        report = dict(domain=args.domain,wall_seconds=args.seconds,
            constants={k:getattr(HazardDetection,k) for k in dir(HazardDetection)
                       if k.isupper() and isinstance(getattr(HazardDetection,k),(int,float,str))},
            samples=dict(samples),hazards=dict(hazards),cliff_scans=streams)
        content = json.dumps(report,indent=2,allow_nan=False)+'\n'
        if args.output:
            args.output.parent.mkdir(parents=True,exist_ok=True)
            args.output.write_text(content)
        print(content)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)


if __name__=='__main__':
    main()
