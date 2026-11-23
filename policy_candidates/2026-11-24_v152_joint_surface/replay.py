"""Compare goal choices on saved live SLAM inputs. The robot is not simulated here."""
import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
from time import perf_counter

import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from sensor_msgs.msg import CameraInfo
from tf2_msgs.msg import TFMessage

from osp_mission import _r32_navigation_core as r32
from osp_mission.box_views import odom_point
from osp_mission.center_wall import CenterWall
from osp_mission.r32_drive import grid_from_message


def quaternion_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def inputs(run, queries):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(run / 'rosbag'), storage_id='mcap'), ConverterOptions('cdr', 'cdr'))
    cost_topic = '/global_costmap/costmap'
    camera_topic = '/oakd/rgb/preview/camera_info'
    types = {cost_topic: OccupancyGrid, '/tf': TFMessage, camera_topic: CameraInfo}
    reader.set_filter(StorageFilter(topics=list(types)))
    cost = transform = camera = None
    cost_time = -math.inf
    query_times = [t for t, _ in queries]
    index = 0
    while reader.has_next() and index < len(queries):
        topic, raw, stamp = reader.read_next()
        now = stamp / 1e9
        while index < len(queries) and now > query_times[index]:
            yield cost if query_times[index] - cost_time <= 3 else None, transform, camera
            index += 1
        message = deserialize_message(raw, types[topic])
        if topic == cost_topic and message.header.frame_id == 'map':
            cost, cost_time = grid_from_message(message), now
        if topic == camera_topic and message.k[0] > 0:
            camera = message.k[0], 2 * math.atan(message.width / (2 * message.k[0]))
        if topic == '/tf':
            for frame in message.transforms:
                if frame.header.frame_id == 'map' and frame.child_frame_id == 'odom':
                    tr = frame.transform.translation
                    a = -quaternion_yaw(frame.transform.rotation)
                    transform = (-math.cos(a) * tr.x + math.sin(a) * tr.y,
                                 -math.sin(a) * tr.x - math.cos(a) * tr.y, a)


def replay(run, output):
    artifacts = run / 'artifacts'
    startup = [json.loads(line) for line in (artifacts / 'startup.jsonl').read_text().splitlines()]
    start = next(e['wall_epoch_s'] for e in startup if e['event'] == 'mission_started')
    audit = [json.loads(line) for line in (artifacts / 'center_wall.jsonl').read_text().splitlines()]
    queries = []
    for p in sorted((artifacts / 'policy_decisions').glob('*.npz')):
        with np.load(p, allow_pickle=False) as data:
            queries.append((start + float(data['elapsed']), p))
    assert len(queries) == len(audit)
    scheduler = CenterWall()
    rows = []
    original = scheduler.joint.choose
    selector_times = []

    def timed_selection(*args, **kwargs):
        start = perf_counter()
        result = original(*args, **kwargs)
        selector_times.append((perf_counter() - start) * 1000)
        return result

    scheduler.joint.choose = timed_selection
    for (_, path), (cost, transform, camera), old in zip(queries, inputs(run, queries), audit):
        if cost is None or camera is None or transform is None:
            continue
        with np.load(path, allow_pickle=False) as d:
            robot = (*d['robot'], float(d['heading']))
            grid = r32.Grid(d['data'], float(d['resolution']), (*d['origin'], 0.))
            elapsed = float(d['elapsed'])
            visits = d['visits']
            candidates = [dict(x=x, y=y, yaw=yaw, scan_angle=angle, distance=features[2],
                               features=np.asarray(features), stationary=False)
                          for x, y, yaw, angle, *features in d['candidates']]
            saved = d['candidates'][int(d['selected_index'])]
            observed = old['kind']
            if observed == 'flow_or_marker' or old.get('survey_escape'):
                continue
            scheduler.survey.camera = camera
            scheduler.batch.update(
                [tuple(old['batch_box'])] if old.get('batch_box') is not None else [],
                [], odom_point(robot[:2], transform),
            )
            def veto(goal):
                return any(math.hypot(goal['x'] - x, goal['y'] - y) < .45
                           for x, y in visits[-8:])
            begin = perf_counter()
            selected = scheduler.decide(grid, cost, robot, transform, d['viewed'],
                                        elapsed, 600., veto, candidates)
            ms = (perf_counter() - begin) * 1000
            if selected is None:
                continue
            changed = math.hypot(saved[0] - selected['x'], saved[1] - selected['y']) > .4
            rows.append(dict(
                elapsed_s=round(elapsed, 2), old_kind=observed, new_kind=selected['r32_kind'],
                new_reason=scheduler.reason, different=changed, old_x=round(float(saved[0]), 3),
                old_y=round(float(saved[1]), 3), new_x=round(selected['x'], 3), new_y=round(selected['y'], 3),
                old_goal_straight_m=round(math.hypot(saved[0] - robot[0], saved[1] - robot[1]), 3),
                new_goal_straight_m=round(math.hypot(selected['x'] - robot[0], selected['y'] - robot[1]), 3),
                estimated_route_m=selected.get('joint_route_m'), estimated_views=selected.get('joint_views'),
                decision_ms=round(ms, 2),
            ))
            scheduler.dispatched(selected, elapsed)
    if not rows:
        raise RuntimeError('No decision replayed')
    with output.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    changed = sum(r['different'] for r in rows)
    print(json.dumps(dict(total_saved=len(queries), compared=len(rows), different=changed,
                          reasons=Counter(r['new_reason'] for r in rows),
                          selector_calls=len(selector_times),
                          selector_median_ms=round(float(np.median(selector_times)), 2) if selector_times else None,
                          selector_p95_ms=round(float(np.percentile(selector_times, 95)), 2) if selector_times else None),
                     ensure_ascii=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    replay(args.run, args.output)
