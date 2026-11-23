"""Audit a finished exam from the passive camera/action recording and saved SLAM decisions."""
import argparse
from collections import Counter, deque
import csv
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'2026-11-22_random_exam'))
from box_metrics import outer_walls
from route_repeat import analyze
from sensor_audit import audit_episode, bag_messages, epochs


def calibration(bag):
    import rosbag2_py as storage
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo
    from tf2_msgs.msg import TFMessage

    reader = storage.SequentialReader()
    reader.open(storage.StorageOptions(uri=str(bag), storage_id='mcap'), storage.ConverterOptions('', ''))
    reader.set_filter(storage.StorageFilter(topics=['/tf_static', '/mission/processed_camera']))
    transforms = {}
    camera = None
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic == '/mission/processed_camera':
            if camera is None:
                camera = deserialize_message(raw, CameraInfo)
            continue
        for tr in deserialize_message(raw, TFMessage).transforms:
            q = tr.transform.rotation
            p = tr.transform.translation
            matrix = np.eye(4)
            matrix[:3, :3] = Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            matrix[:3, 3] = [p.x,p.y,p.z]
            transforms[(tr.header.frame_id.strip('/'), tr.child_frame_id.strip('/'))] = matrix
    reader.close()
    if camera is None:
        raise ValueError('No processed CameraInfo')
    target = camera.header.frame_id.strip('/')
    queue = deque([('base_link', np.eye(4))])
    visited = {'base_link'}
    while queue:
        node, matrix = queue.popleft()
        if node == target:
            return dict(frame=target, width=camera.width, height=camera.height, k=list(camera.k),
                        quaternion=Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist(),
                        translation=matrix[:3, 3].tolist())
        for (parent, child), transform in transforms.items():
            if node not in (parent,child):
                continue
            to = child if parent == node else parent
            if to not in visited:
                visited.add(to)
                queue.append((to, matrix @ (transform if parent == node else np.linalg.inv(transform))))
    raise ValueError(f'No static transform from base_link to {target}')


def summarize(run, board_root):
    read = lambda name: json.loads((run/name).read_text())
    score = read('exam-score.json')
    protocol = read('protocol.json')
    camera = read('camera_metrics.json')
    if protocol['physical_drive'] or protocol['camera'] != [640,480,5]:
        raise ValueError('Requires a 640x480, 5 fps simulated exam')
    decision_file = run/'native/center_wall.jsonl'
    decisions = [json.loads(row) for row in decision_file.read_text().splitlines() if row]
    with (run/'headings.csv').open() as stream:
        motion = np.asarray(list(csv.reader(stream))[1:], dtype=float)
    t, x, y, yaw = motion.T
    time = np.arange(t[0], t[-1], .25)
    speed = np.hypot(np.diff(np.interp(time,t,x)), np.diff(np.interp(time,t,y))) / .25
    turn = np.abs(np.diff(np.interp(time,t,np.unwrap(yaw)))) / .25
    bag = board_root/'sensor-recordings'/run.name
    result = dict(seed=protocol['seed'], fixture_sha256=protocol['fixture_sha256'],
                  workspace=protocol['workspace'], run=run.name, score=score, camera=camera,
                  turns_at_rest_s=float(((speed<.035)&(turn>.15)).sum()*.25),
                  almost_still_s=float(((speed<.035)&(turn<=.15)).sum()*.25),
                  dispatches=len(decisions), kinds=Counter(d['kind'] for d in decisions))
    if not (bag/'metadata.yaml').exists():
        result['coverage_unavailable'] = 'No finalized passive action and camera recording'
        return result
    try:
        cal = calibration(bag)
        evidence, = epochs(bag_messages(bag))
        sensor = audit_episode(run, evidence, cal)
        result['sensors'] = sensor
        if sensor['valid']:
            result['outer_walls'] = outer_walls(read('scene.json'),sensor['surface_coverage'])
            result['route'] = analyze(run, board_root, cal, sensor)['recent']
    except (ValueError, KeyError, OSError, RuntimeError) as exc:
        result['coverage_unavailable'] = str(exc)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('board_root', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = summarize(args.run, args.board_root)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
