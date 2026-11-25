"""Passively measure actual native Gazebo sensor topics in simulation seconds."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, LaserScan

from audit import timing


def main():
    import os
    if os.environ.get('ROS_DOMAIN_ID') != '185':
        raise RuntimeError('Sensor probe is simulation-only')
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    rclpy.init()
    node = Node('real_sim_sensor_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    stamps, received, shapes = {}, {}, {}
    def observe(topic, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        stamps.setdefault(topic, []).append(t)
        received.setdefault(topic, []).append(node.get_clock().now().nanoseconds/1e9)
        if isinstance(msg, Image):
            shapes['image'] = [msg.width, msg.height]
        elif isinstance(msg, CameraInfo):
            shapes['camera_info'] = [msg.width, msg.height]
            shapes['k'] = list(msg.k)
        else:
            shapes['lidar'] = dict(beams=len(msg.ranges), raw_max_range_m=msg.range_max,
                                  angle_min=msg.angle_min, angle_max=msg.angle_max)
    for name, kind in [('/scan', LaserScan), ('/oakd/rgb/preview/image_raw', Image),
                       ('/oakd/rgb/preview/camera_info', CameraInfo)]:
        node.create_subscription(kind, name, lambda msg, name=name: observe(name, msg), qos_profile_sensor_data)
    start = time.monotonic()
    while time.monotonic()-start < 120:
        rclpy.spin_once(node, timeout_sec=.1)
        if len(stamps) == 3 and min(ts[-1]-ts[0] for ts in stamps.values()) >= 20.:
            break
    result = dict(physical_drive=False, native_gazebo=True, motion_command_sent=False,
                  shapes=shapes, topics={k: timing(v, received[k]) for k, v in stamps.items()},
                  complete=len(stamps) == 3 and min(ts[-1]-ts[0] for ts in stamps.values()) >= 20.)
    a.output.write_text(json.dumps(result, indent=2)+'\n')
    node.destroy_node()
    rclpy.shutdown()
    if not result['complete']:
        raise RuntimeError('Incomplete native sensor probe')


if __name__ == '__main__':
    main()
