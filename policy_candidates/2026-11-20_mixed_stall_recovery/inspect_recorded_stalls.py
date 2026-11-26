"""Inspect stationary navigation cancellations using recorded scans and exposure TF."""
import argparse
import json
from pathlib import Path
import re

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer
from osp_mission.scan_safety import rear_clear


def messages(bag, names):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {row.name: get_message(row.type) for row in reader.get_all_topics_and_types()
             if row.name in names}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    while reader.has_next():
        name, raw, timestamp = reader.read_next()
        yield name, deserialize_message(raw, types[name]), timestamp / 1e9


def inspect(run):
    text = (run / 'console.log').read_text(errors='replace')
    times = [float(t) for t in re.findall(r'\[([\d.]+)\] \[mission\]: Cancel nav: navigation_stalled', text)]
    tf = Buffer(cache_time=Duration(seconds=900))
    for name, msg, _ in messages(run / 'rosbag', {'/tf', '/tf_static'}):
        setter = tf.set_transform_static if name == '/tf_static' else tf.set_transform
        for transform in msg.transforms:
            setter(transform, 'recording')
    odom, scans, commands, collisions = [], [], {}, []
    names = {'/odom', '/scan_self_filtered', '/scan', '/cmd_vel', '/cmd_vel_nav',
             '/cmd_vel_smoothed', '/collision_monitor_state'}
    for name, msg, received in messages(run / 'rosbag', names):
        if name == '/odom':
            p = msg.pose.pose.position
            odom.append((received, p.x, p.y, msg.twist.twist.linear.x))
        elif name in ('/scan_self_filtered', '/scan'):
            if not any(abs(received - t) <= .6 for t in times):
                continue
            try:
                pose = tf.lookup_transform('base_link', msg.header.frame_id,
                                           Time.from_msg(msg.header.stamp)).transform
                q = pose.rotation
                yaw = np.arctan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                clear = rear_clear(msg, (pose.translation.x, pose.translation.y, yaw))
                scans.append(dict(time=received, topic=name, rear_clear=bool(clear)))
            except Exception as exc:
                scans.append(dict(time=received, topic=name, error=str(exc)))
        elif name == '/collision_monitor_state':
            collisions.append((received, int(msg.action_type), msg.polygon_name))
        else:
            twist = msg.twist if hasattr(msg, 'twist') else msg
            commands.setdefault(name, []).append((received, twist.linear.x, twist.angular.z))
    out = []
    for t in times:
        rows = [r for r in odom if t-8 <= r[0] <= t]
        displacement = max((np.hypot(r[1]-rows[0][1], r[2]-rows[0][2]) for r in rows), default=None)
        near_scans = [r for r in scans if abs(r['time']-t) <= .6]
        filtered = [r for r in near_scans if r['topic'] == '/scan_self_filtered']
        closest = min(filtered or near_scans, key=lambda r:abs(r['time']-t), default=None)
        out.append(dict(stamp=t, displacement_8s_m=displacement,
                        sensor_scan=closest,
                        command_mean_abs={name:[float(np.mean([abs(r[i]) for r in rs if t-8<=r[0]<=t])) if any(t-8<=r[0]<=t for r in rs) else None
                                                for i in (1,2)] for name,rs in commands.items()},
                        collision_states=sorted({str(r[1:]) for r in collisions if t-8<=r[0]<=t})))
    return dict(run=run.name, stalls=out,
                logged_backup_actions=text.count('Running backup'),
                safety_stop='Hardware cliff/stall/wheel-drop hazard' in text)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    result = [inspect(run) for run in args.runs]
    args.output.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    rclpy.shutdown()
