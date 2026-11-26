"""Replay recorded camera, scans and exposure poses through Detector.on_rgb offline."""
import argparse
from collections import Counter
import json
import os
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from osp_mission.detector import Detector


def messages(bag, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        yield topic, deserialize_message(raw, topics[topic])


class Capture:
    def __init__(self):
        self.rows = []

    def publish(self, msg):
        self.rows.append(json.loads(msg.data))

    def get_subscription_count(self):
        return 1


def replay(bag):
    rclpy.init()
    node = Detector()
    buffer = Buffer(cache_time=Duration(seconds=900))
    rgb = '/oakd/rgb/preview/image_raw'
    info = '/oakd/rgb/preview/camera_info'
    processed = '/mission/processed_camera'
    def stamp(msg):
        return (msg.header.stamp.sec, msg.header.stamp.nanosec)
    stamps = set()
    has_map = False
    for topic, msg in messages(bag, {'/tf': TFMessage, '/tf_static': TFMessage, processed: CameraInfo}):
        if topic == processed:
            stamps.add(stamp(msg))
            continue
        for tr in msg.transforms:
            has_map |= tr.header.frame_id == 'map' and tr.child_frame_id == 'odom'
            setter = buffer.set_transform_static if topic == '/tf_static' else buffer.set_transform
            setter(tr, 'recording')
    node.tf = buffer
    if not has_map:
        from rclpy.parameter import Parameter
        node.set_parameters([Parameter('map_frame', value='odom')])
    # Suppress all outbound publications. Only camera callbacks run.
    class Sink:
        def publish(self, msg): pass
        def get_subscription_count(self): return 0
    for attr in ('processed_camera','candidate_pub','small_wall_pub','print_pub','glance_pub','pub','diagnostic_pub'):
        setattr(node, attr, Sink())
    node.bearing_pub, node.observation_pub = Capture(), Capture()
    for topic, msg in messages(bag, {rgb: Image, info: CameraInfo, '/scan': LaserScan}):
        if topic == info: node.on_info(msg)
        elif topic == '/scan': node.on_scan(msg)
        elif stamp(msg) in stamps: node.on_rgb(msg)
    # Use the same published point markers and frame filter as the board.
    class Targets:
        def __init__(self):
            self.ids = None

        def publish(self, array):
            self.ids = sorted({marker.id for marker in array.markers
                               if marker.ns == 'point' and marker.action == 0
                               and marker.header.frame_id == node.p('map_frame')})

    targets = Targets()
    node.pub = targets
    node.publish()
    if targets.ids is None:
        raise RuntimeError('Detector did not publish a target array')
    out = dict(frame=node.p('map_frame'), decoded=node.diagnostics['decoded'],
               confirmed=targets.ids, confirmations={i:len(rows) for i, rows in node.observations.items()},
               diagnostics=node.diagnostics, cues=node.bearing_pub.rows,
               observation_sources=dict(Counter(r.get('range_source','baseline') for r in node.observation_pub.rows)),
               observations=[{k:r[k] for k in ('id','stamp','map_point') if k in r} for r in node.observation_pub.rows])
    node.destroy_node()
    rclpy.shutdown()
    return out


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('bag',type=Path)
    parser.add_argument('output',type=Path)
    args = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') in (None,'4'):
        parser.error('Use an isolated local ROS domain for offline replay')
    result=replay(args.bag)
    args.output.write_text(json.dumps(result,indent=2))
    print({k: result[k] for k in ('frame','decoded','confirmed','confirmations','diagnostics','observation_sources')})
