"""Offline decision comparison on recorded map snapshots using the last available costmap."""
import argparse
import bisect
import json
from pathlib import Path
import time

import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid
from osp_mission.r32_drive import grid_from_message
from osp_mission._r32_navigation_core import Grid
from osp_mission.center_wall import CenterWall


def run(root):
    artifacts=root/'artifacts'
    events=[json.loads(s) for s in (artifacts/'startup.jsonl').read_text().splitlines()]
    start=next(e['wall_epoch_s'] for e in events if e['event']=='mission_started')
    bag=SequentialReader()
    bag.open(StorageOptions(uri=str(root/'rosbag'), storage_id='mcap'),ConverterOptions('',''))
    bag.set_filter(StorageFilter(topics=['/global_costmap/costmap']))
    stamps,costs=[],[]
    while bag.has_next():
        _,raw,received=bag.read_next()
        msg=deserialize_message(raw,OccupancyGrid)
        stamps.append(received/1e9)
        costs.append((msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,grid_from_message(msg)))
    rows=[]
    schedule=CenterWall()
    for file in sorted((artifacts/'policy_decisions').glob('*.npz')):
        with np.load(file) as z:
            elapsed=float(z['elapsed'])
            when=start+elapsed
            idx=bisect.bisect_right(stamps,when)-1
            if idx<0 or not -.1<=when-costs[idx][0]<=3.:
                rows.append(dict(map=file.stem,skipped='costmap_age'))
                continue
            grid=Grid(z['data'],float(z['resolution']),(*z['origin'],0.))
            t=time.monotonic()
            goal=schedule.decide(grid,costs[idx][1],(*z['robot'],float(z['heading'])),
                                 (0.,0.,0.),z['viewed'],elapsed,600.,lambda _:False)
            rows.append(dict(map=file.stem,t=round(elapsed,1),phase=schedule.phase,
                             kind=goal['r32_kind'] if goal else None,
                             target=goal['surface_map'] if goal else None,
                             compute_ms=round((time.monotonic()-t)*1000)))
            if goal:
                schedule.dispatched(goal,elapsed)
    return rows


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('run',type=Path)
    print(json.dumps(run(p.parse_args().run)))
