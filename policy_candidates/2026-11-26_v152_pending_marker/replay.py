"""Replay saved preview pixels and the first ranged candidate on the next planning snapshot."""
import argparse
from collections import Counter
import json
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from osp_mission.marker_focus import MarkerFocus
from osp_mission.marker_pair import decode


def replay(run):
    events = [json.loads(s) for s in (run / 'artifacts/startup.jsonl').read_text().splitlines()]
    start = next(e['wall_epoch_s'] for e in events if e['event'] == 'mission_started')
    image_topic = '/oakd/rgb/preview/image_raw/compressed'
    cue_topic = '/mission/marker_candidates'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(run / 'rosbag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[image_topic, cue_topic]))
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    n, weak_frames, shape = 0, 0, None
    small_ids, larger_ids = Counter(), Counter()
    first = None
    while reader.has_next():
        topic, raw, received = reader.read_next()
        if topic == cue_topic:
            if first is None:
                first = (json.loads(deserialize_message(raw, String).data), received / 1e9)
            continue
        image = cv2.imdecode(np.frombuffer(deserialize_message(raw, CompressedImage).data, np.uint8),
                             cv2.IMREAD_GRAYSCALE)
        n += 1
        shape = image.shape
        weak = []
        corners, ids = decode(image, dictionary, weak)
        weak_frames += bool(weak)
        for q, ident in zip(corners, ids.ravel() if ids is not None else []):
            edge = np.linalg.norm(q[0] - np.roll(q[0], 1, axis=0), axis=1).min()
            (small_ids if edge < 10. else larger_ids)[int(ident)] += 1
    report = dict(frames=n, shape=shape, anonymous_small_frames=weak_frames,
                  decoded_below_10=small_ids, decoded_at_least_10=larger_ids)
    if first is None:
        return report
    cue, received = first
    for file in sorted((run / 'artifacts/policy_decisions').glob('*.npz')):
        with np.load(file) as a:
            if start + float(a['elapsed']) < received:
                continue
            choices = [dict(x=x, y=y, yaw=h, scan_angle=s, distance=d,
                            features=np.array([v,f,d,t,r])) for x,y,h,s,v,f,d,t,r in a['candidates']]
            rows = []
            # Vary the callback order using the same measured geometry at each offset.
            for age in (.15, .50, .99, 1.20):
                focus = MarkerFocus()
                assert focus.observe(cue, cue['camera_origin_map'][:2], received)
                goal = focus.choose(a['data'], float(a['resolution']), tuple(a['origin']),
                                    a['robot'], float(a['heading']), choices, {}, cue['stamp'] + age)
                rows.append(dict(age_s=age, id=None if goal is None else goal['focus_id'],
                                 travel_m=None if goal is None else round(goal['distance'], 3)))
            report.update(candidate_id=cue['id'], candidate_px=cue['min_edge_pixels'],
                          candidate_capture_s=cue['stamp']-start, snapshot=file.name, focus=rows)
            break
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    print(json.dumps(replay(args.run), indent=2))
