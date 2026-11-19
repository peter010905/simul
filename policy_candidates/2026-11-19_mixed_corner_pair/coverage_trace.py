"""Observe the live Mixed box coverage counters without publishing any ROS messages."""
import csv
import json
import os
from pathlib import Path
import time

import rclpy
from std_msgs.msg import String


def record(folder):
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('ROS_DOMAIN_ID') not in ('174','175'):
        raise RuntimeError('Requires isolated simulation domain')
    rclpy.init()
    node=rclpy.create_node('matched_box_coverage_observer')
    try:
        with (folder/'box_coverage.csv').open('x',buffering=1) as stream:
            rows=csv.writer(stream)
            rows.writerow(['mission_elapsed_s','confirmed_boxes','observed_faces','two_view_frames'])
            def status(msg):
                data=json.loads(msg.data)
                camera=data.get('camera_diagnostics',{})
                if data.get('state')=='running' and 'two_view_frames' in camera:
                    rows.writerow([data['elapsed_seconds'],camera['confirmed_boxes'],
                                   camera['observed_box_faces'],camera['two_view_frames']])
            node.create_subscription(String,'/mission/status',status,10)
            deadline=time.monotonic()+7200
            while time.monotonic()<deadline and not (folder/'metrics.json').is_file():
                rclpy.spin_once(node,timeout_sec=.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    import sys
    record(Path(sys.argv[1]))
