"""Compare image decoding on original OAK-D frames. No ROS publishers."""
import argparse
from collections import Counter
import json
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np
import rosbag2_py as bag
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from osp_mission.marker_pair import decode


def replay(run):
    reader = bag.SequentialReader()
    reader.open(bag.StorageOptions(uri=str(run / 'rosbag'), storage_id='mcap'),
                bag.ConverterOptions('', ''))
    reader.set_filter(bag.StorageFilter(topics=['/oakd/rgb/preview/image_raw']))
    bridge = CvBridge()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    totals = [Counter(), Counter()]
    pairs = [0, 0]
    timings = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        image = bridge.imgmsg_to_cv2(deserialize_message(data, Image), 'mono8')
        old_corners, old_ids, _ = cv2.aruco.detectMarkers(image, dictionary)
        start = perf_counter()
        new_result = decode(image, dictionary)
        timings.append((perf_counter() - start) * 1000)
        for i, (corners, ids) in enumerate(((old_corners, old_ids), new_result)):
            seen = set()
            for quad, ident in zip(corners, ids.ravel() if ids is not None else ()):
                lengths = np.linalg.norm(quad[0] - np.roll(quad[0], 1, axis=0), axis=1)
                if min(lengths) >= 18:
                    seen.add(int(ident))
            totals[i].update(seen)
            pairs[i] += {103, 7} <= seen
    print(json.dumps(dict(run=run.name, frames=len(timings), baseline=totals[0],
                          candidate=totals[1], both=pairs, p95_ms=np.percentile(timings, 95))), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', type=Path)
    cv2.setNumThreads(1)
    for path in parser.parse_args().runs:
        replay(path)
