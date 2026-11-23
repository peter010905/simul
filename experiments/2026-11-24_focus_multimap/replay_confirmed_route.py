"""Inspect the first planning decision after a registered print using recorded sensor data."""
import argparse
import json
from pathlib import Path

import numpy as np
import rosbag2_py as bag
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from osp_mission import _r32_navigation_core as r32
from osp_mission.edge_policy import visible
from osp_mission.marker_approach import MarkerApproach
from scipy.ndimage import distance_transform_edt


def measure(run, observation_bag, ident):
    score = json.loads((run / 'exam-score.json').read_text())
    registered = score['records'][str(ident)]['registered_s']
    start = next(e['sim'] for e in map(json.loads, (run/'events.jsonl').read_text().splitlines())
                 if e['event'] == 'start_ack')
    for path in sorted((run / 'native/policy_decisions').glob('*.npz')):
        with np.load(path) as snapshot:
            if snapshot['elapsed'] >= registered:
                elapsed = float(snapshot['elapsed'])
                grid = r32.Grid(snapshot['data'], float(snapshot['resolution']), (*snapshot['origin'], 0.))
                pose = (*snapshot['robot'], float(snapshot['heading']))
                break
    else:
        raise ValueError('No planning snapshot after registration')

    observations, targets = [], []
    reader = bag.SequentialReader()
    reader.open(bag.StorageOptions(uri=str(observation_bag), storage_id='mcap'), bag.ConverterOptions('', ''))
    reader.set_filter(bag.StorageFilter(topics=['/mission/marker_observations', '/targets']))
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic == '/mission/marker_observations':
            d = json.loads(deserialize_message(raw, String).data)
            if d['id'] == ident and d['stamp'] <= start + elapsed and d.get('camera_origin_map'):
                observations.append(d)
        else:
            for marker in deserialize_message(raw, MarkerArray).markers:
                stamp = marker.header.stamp.sec + marker.header.stamp.nanosec/1.e9
                if marker.id == ident and marker.ns == 'point' and stamp <= start + elapsed:
                    p = marker.pose.position
                    targets.append((stamp, np.array((p.x, p.y))))
    if not targets or not observations:
        raise ValueError('Missing camera observation or registered map position')
    point = max(targets, key=lambda pair: pair[0])[1]
    exposure = max(observations, key=lambda d: d['stamp'])
    direction = np.array(exposure['camera_origin_map'][:2]) - point
    safe, distances = grid.reachable(pose[:2], .32)
    direct = np.where((grid.cells == 0) & (distance_transform_edt(grid.cells <= 50)*grid.resolution >= .32), 0, 100)
    routes, straight = [], 0
    for _, position in MarkerApproach.near_views(point, direction):
        cell = grid.index(*position)
        delta = position - point
        end = grid.index(*(point + .25 * delta / np.linalg.norm(delta)))
        if (not grid.inside(*cell) or not safe[cell] or not np.isfinite(distances[cell]) or
                not visible(grid.cells, *cell, np.array([end[0]]), np.array([end[1]]))[0]):
            continue
        routes.append(float(distances[cell]))
        a, b = (pose[:2]-np.asarray(grid.origin[:2]))/grid.resolution, (position-grid.origin[:2])/grid.resolution
        straight += int(visible(direct, a[1], a[0], np.array([b[1]]), np.array([b[0]]))[0])
    return dict(marker=ident, registration_s=registered, decision_s=elapsed, candidates=15,
                connected_slam=len(routes), direct_clear=straight, shortest_slam_m=min(routes, default=None),
                camera_stamp=exposure['stamp'], recorded_target_map=point.tolist(),
                limits='Uses the recorded optical origin as a proxy for observer base. No historical Nav2 costmap or failed-goal list.')


if __name__ == '__main__':
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('run', type=Path)
    cli.add_argument('observations', type=Path)
    cli.add_argument('id', type=int)
    args = cli.parse_args()
    print(json.dumps(measure(args.run, args.observations, args.id), indent=2, allow_nan=False))
