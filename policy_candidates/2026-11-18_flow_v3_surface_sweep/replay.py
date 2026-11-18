"""Compare centered and extended aims at the logged surface turns; no ROS output."""
import argparse
from bisect import bisect_right
import math
from pathlib import Path
import re

import rosbag2_py as bag
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer

from osp_mission.surface_sweep import SurfaceSweep, extend_turn
from osp_mission.wall_route import camera_pose, wrap, yaw


def replay(run):
    events = [(float(t), int(angle)) for t, angle in re.findall(
        r'\[(\d+\.\d+)\] \[mission\]: Facing potential marker surface: (-?\d+) deg',
        (run/'console.log').read_text())]
    reader = bag.SequentialReader()
    reader.open(bag.StorageOptions(uri=str(run/'rosbag'), storage_id='mcap'), bag.ConverterOptions('', ''))
    topics = ['/scan_self_filtered', '/tf_static', '/odom', '/mission/processed_camera']
    types = {x.name:get_message(x.type) for x in reader.get_all_topics_and_types() if x.name in topics}
    reader.set_filter(bag.StorageFilter(topics=topics))
    static = Buffer()
    scans, poses, infos = [], [], []
    while reader.has_next():
        topic, data, ns = reader.read_next()
        msg = deserialize_message(data, types[topic])
        t = ns/1e9
        if topic == '/tf_static':
            for tr in msg.transforms:
                static.set_transform_static(tr, 'bag')
        elif topic == '/odom':
            p = msg.pose.pose
            poses.append((t, (p.position.x, p.position.y, yaw(p.orientation))))
        elif topic == '/mission/processed_camera':
            infos.append((t, msg))
        else:
            scans.append((t, msg))

    def at(rows, t):
        idx = bisect_right(rows, t, key=lambda row:row[0]) - 1
        if idx >= 0 and t - rows[idx][0] < .6:
            return rows[idx][1]
        return None

    def project(base, offset):
        x, y, rot = base
        dx, dy, heading = offset
        return (x + math.cos(rot)*dx - math.sin(rot)*dy,
                y + math.sin(rot)*dx + math.cos(rot)*dy, wrap(rot + heading))

    survey = SurfaceSweep()
    print('event,logged_center,replayed_center,extended_degrees')
    for i, (t, logged) in enumerate(events):
        scan, base, info = at(scans,t), at(poses,t), at(infos,t)
        if scan is None or base is None or info is None:
            continue
        laser = static.lookup_transform('base_link',scan.header.frame_id,Time()).transform
        cam = project(base,camera_pose(static.lookup_transform('base_link',info.header.frame_id,Time()).transform))
        fov = 2*math.atan(info.width / (2*info.k[0]))
        center = survey.select(scan, project(base,(laser.translation.x, laser.translation.y, yaw(laser.rotation))), cam, fov, t)
        if center is None:
            continue
        direction = math.atan2(center[1]-cam[1],center[0]-cam[0])
        angle = wrap(direction-cam[2])
        final = extend_turn(angle,survey.tip,cam,fov)
        survey.recent.append((t,center,direction))
        print(i+1,logged,round(math.degrees(angle),1),round(math.degrees(final),1),sep=',')
    print('logged_events',len(events))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    replay(parser.parse_args().run)
