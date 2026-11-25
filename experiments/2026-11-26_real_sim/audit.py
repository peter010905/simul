"""Read finalized physical MCAPs offline; never create a ROS node or publish data.

Reception timestamps measure this recorder path, not sensor-only transport latency.
Wheel odometry is not independent ground truth for slip or payload identification.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml


def quantiles(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    return dict(zip(('p05', 'p50', 'p95'), map(float, np.quantile(a, [.05, .5, .95])))) if len(a) else None


def timing(stamps, received):
    t = np.asarray(stamps, dtype=float)
    dt = np.diff(t)
    positive = dt[dt > 0]
    return dict(count=len(t), period_s=quantiles(positive),
                stamp_span_s=float(t[-1] - t[0]) if len(t) else None,
                reception_span_s=float(received[-1] - received[0]) if len(t) else None,
                median_hz=float(1 / np.median(positive)) if len(positive) else None,
                nonpositive_intervals=int(np.sum(dt <= 0)),
                gaps_over_0_5s=int(np.sum(dt > .5)),
                recorded_age_s=quantiles(np.asarray(received) - t))


def audit(folder):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    folder = Path(folder)
    metadata = yaml.safe_load((folder / 'rosbag/metadata.yaml').read_text())['rosbag2_bagfile_information']
    run = json.loads((folder / 'run.json').read_text())
    slam_path = folder / 'config/slam.yaml'
    slam = yaml.safe_load(slam_path.read_text())['slam_toolbox']['ros__parameters']
    types = {x['topic_metadata']['name']: x['topic_metadata']['type']
             for x in metadata['topics_with_message_count']}
    names = ['/scan', '/scan_self_filtered', '/odom', '/cmd_vel', '/map',
             '/oakd/rgb/preview/camera_info', '/mission/processed_camera',
             '/oakd/rgb/preview/image_raw/compressed']
    names = [n for n in names if n in types]
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(folder / 'rosbag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=names))
    classes = {n: get_message(types[n]) for n in names}
    stamps = {n: [] for n in names}
    received = {n: [] for n in names}
    cameras, scans, maps, motions, image_shapes = {}, {}, {}, {}, {}
    while reader.has_next():
        topic, raw, ts = reader.read_next()
        msg = deserialize_message(raw, classes[topic])
        if not hasattr(msg, 'header'):
            continue
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        stamps[topic].append(stamp)
        received[topic].append(ts * 1e-9)
        if types[topic].endswith('/CameraInfo'):
            key = (msg.width, msg.height, tuple(msg.k), tuple(msg.d), msg.distortion_model)
            row = cameras.setdefault(topic, Counter())
            row[key] += 1
        elif types[topic].endswith('/CompressedImage') and topic not in image_shapes:
            import cv2
            image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
            image_shapes[topic] = list(image.shape) if image is not None else None
        elif types[topic].endswith('/LaserScan'):
            row = scans.setdefault(topic, dict(range_max=Counter(), geometry=Counter(),
                finite_fraction=[], beyond_slam_fraction=[], observed_max_m=[]))
            a = np.asarray(msg.ranges)
            ok = np.isfinite(a) & (a >= msg.range_min) & (a <= msg.range_max)
            row['range_max'][float(msg.range_max)] += 1
            row['geometry'][(len(a), float(msg.angle_min), float(msg.angle_max),
                             float(msg.scan_time), float(msg.time_increment))] += 1
            row['finite_fraction'].append(float(ok.mean()))
            row['beyond_slam_fraction'].append(float(np.mean(ok & (a > slam['max_laser_range']))))
            row['observed_max_m'].append(float(a[ok].max()) if ok.any() else float('nan'))
        elif topic == '/map':
            data = np.asarray(msg.data, dtype=np.int8)
            signature = (msg.info.width, msg.info.height, msg.info.resolution,
                         msg.info.origin.position.x, msg.info.origin.position.y,
                         hashlib.blake2b(data.tobytes(), digest_size=8).hexdigest())
            row = maps.setdefault(topic, dict(changes=0, last=None, changed_stamps=[], resolutions=Counter()))
            row['resolutions'][float(msg.info.resolution)] += 1
            if signature != row['last']:
                row['changes'] += 1
                row['changed_stamps'].append(stamp)
                row['last'] = signature
        elif topic in ('/cmd_vel', '/odom'):
            twist = msg.twist.twist if topic == '/odom' else msg.twist
            motions.setdefault(topic, []).append([twist.linear.x, twist.angular.z])
    bag_start = metadata['starting_time']['nanoseconds_since_epoch']/1e9
    bag_end = bag_start+metadata['duration']['nanoseconds']/1e9
    result = dict(run=folder.name, workspace=run.get('workspace'),
        duration_s=metadata['duration']['nanoseconds'] / 1e9,
        config_snapshot_sha256=hashlib.sha256(slam_path.read_bytes()).hexdigest(),
        slam={k: slam.get(k) for k in ('max_laser_range', 'resolution', 'minimum_time_interval',
            'minimum_travel_distance', 'minimum_travel_heading', 'map_update_interval',
            'check_min_dist_and_heading_precisely')},
        timing={n: timing(stamps[n], received[n]) for n in names}, cameras={}, scans={}, maps={}, motions={},
        first_compressed_image_shapes=image_shapes)
    for n in names:
        if received[n]:
            result['timing'][n]['first_reception_after_bag_start_s'] = received[n][0]-bag_start
            result['timing'][n]['last_reception_before_bag_end_s'] = bag_end-received[n][-1]
    for n, counter in cameras.items():
        result['cameras'][n] = [dict(width=k[0], height=k[1], k=list(k[2]), d=list(k[3]),
            distortion=k[4], count=count) for k, count in counter.most_common(3)]
    for n, row in scans.items():
        result['scans'][n] = dict(range_max_counts=dict(row['range_max']),
            geometries=[dict(beams=k[0], angle_min=k[1], angle_max=k[2], scan_time_s=k[3],
                time_increment_s=k[4], count=v) for k, v in row['geometry'].most_common(3)],
            **{k: quantiles(row[k]) for k in ('finite_fraction', 'beyond_slam_fraction', 'observed_max_m')})
    for n, row in maps.items():
        result['maps'][n] = dict(changes=row['changes'], published=len(stamps[n]),
            change_period_s=quantiles(np.diff(row['changed_stamps'])), resolutions=dict(row['resolutions']))
    for n, rows in motions.items():
        a = np.asarray(rows)
        result['motions'][n] = dict(absolute_linear_m_s=quantiles(abs(a[:, 0])),
                                   absolute_angular_rad_s=quantiles(abs(a[:, 1])))
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('runs', nargs='+')
    a = p.parse_args()
    results = []
    for run in a.runs:
        results.append(audit(a.root / run))
        a.output.write_text(json.dumps(results, indent=2, allow_nan=False) + '\n')
        print('audited', run, results[-1]['slam']['max_laser_range'], flush=True)


if __name__ == '__main__':
    main()
