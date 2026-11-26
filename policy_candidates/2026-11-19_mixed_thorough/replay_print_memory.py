"""Compare print grouping on a recorded bag without issuing navigation commands."""
import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


def memory(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.PrintMemory()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path)
    parser.add_argument('baseline_hybrid', type=Path)
    parser.add_argument('candidate_hybrid', type=Path)
    args = parser.parse_args()
    memories = {"baseline": memory(args.baseline_hybrid, 'baseline_hybrid'),
                "candidate": memory(args.candidate_hybrid, 'candidate_hybrid')}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/mission/anonymous_print']))
    entries = 0
    first = {key: None for key in memories}
    while reader.has_next():
        _, payload, _ = reader.read_next()
        event = json.loads(deserialize_message(payload, String).data)
        stamp = float(event['stamp'])
        entries += 1
        for key, instance in memories.items():
            instance.observe(event, stamp)
            if first[key] is None and any(t['count'] >= 3 for t in instance.tracks):
                first[key] = stamp
    print(json.dumps({"events": entries, "first_three_hit_stamp": first,
                      "eligible_tracks_at_end": {key: sum(t['count'] >= 3 for t in instance.tracks)
                                                 for key, instance in memories.items()},
                      "track_counts_at_end": {key: len(instance.tracks) for key, instance in memories.items()}},
                     indent=2))


if __name__ == '__main__':
    main()
