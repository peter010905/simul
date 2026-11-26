"""Read-only RGB comparison for edge outline candidates in a recorded bag."""
import argparse
import importlib.util
import json
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.borders


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path)
    parser.add_argument('original', type=Path)
    parser.add_argument('candidate', type=Path)
    parser.add_argument('--max-frames', type=int, default=800)
    args = parser.parse_args()
    original, candidate = load(args.original, 'original_borders'), load(args.candidate, 'candidate_borders')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/oakd/rgb/preview/image_raw']))
    bridge = CvBridge()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    detector = cv2.aruco.ArucoDetector(dictionary) if hasattr(cv2.aruco, 'ArucoDetector') else None
    counts = dict(frames=0, aruco_frames=0, original_candidate_frames=0,
                  thorough_candidate_frames=0, extra_candidate_frames=0,
                  original_near_decoded=0, thorough_near_decoded=0,
                  extra_without_decoded=0, original_candidates=0, thorough_candidates=0)
    last = -float('inf')
    while reader.has_next() and counts['frames'] < args.max_frames:
        _, payload, _ = reader.read_next()
        msg = deserialize_message(payload, Image)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp-last < .5:
            continue
        last = stamp
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        old, new = original(frame), candidate(frame)
        corners, ids, _ = (detector.detectMarkers(frame) if detector is not None
                           else cv2.aruco.detectMarkers(frame, dictionary))
        centers = [np.asarray(p).reshape(-1, 2).mean(axis=0) for p in corners]
        near = lambda found: any(np.linalg.norm(np.asarray(x[:2])-c) < max(15, x[2])
                                  for x in found for c in centers)
        counts['frames'] += 1
        counts['aruco_frames'] += bool(ids is not None)
        counts['original_candidate_frames'] += bool(old)
        counts['thorough_candidate_frames'] += bool(new)
        counts['extra_candidate_frames'] += bool(new) and not bool(old)
        counts['original_near_decoded'] += near(old)
        counts['thorough_near_decoded'] += near(new)
        counts['extra_without_decoded'] += bool(new) and not bool(old) and ids is None
        counts['original_candidates'] += len(old)
        counts['thorough_candidates'] += len(new)
    print(json.dumps(counts, indent=2))


if __name__ == '__main__':
    main()
