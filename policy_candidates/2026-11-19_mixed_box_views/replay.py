"""Replay SLAM decision maps; annotate suggestions with held-out world truth afterwards."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from osp_mission import _r32_navigation_core as r32
from osp_mission.box_views import BoxViews, odom_point


def replay(folder):
    scene=json.loads((folder/'scene.json').read_text())
    spawn=scene['robot_spawn']
    world_transform=(spawn['x'],spawn['y'],spawn['yaw'])
    boxes={b['name']:np.array([b['x'],b['y']]) for b in scene['obstacles']}
    poses=sorted((folder/'native/policy_decisions').glob('*.npz'))
    memory=BoxViews()
    proposed=[]
    for p in poses:
        with np.load(p) as data:
            grid=r32.Grid(data['data'],float(data['resolution']),(*data['origin'][:2],0.))
            pose=(*data['robot'],float(data['heading']))
            elapsed=float(data['elapsed'])
        # The replay has no historical Nav2 costmap. Use known SLAM free cells
        # as a proxy here; the installed policy also requires the live costmap.
        goal=memory.choose(grid,grid,pose,(0.,0.,0.),elapsed,{},lambda g:False)
        if goal is None:
            continue
        memory.dispatched(goal,elapsed)
        world=odom_point(goal['face_center_odom'],world_transform)
        name=min(boxes,key=lambda k:np.linalg.norm(world-boxes[k]))
        error=float(np.linalg.norm(world-boxes[name]))
        observer=odom_point((goal['x'],goal['y']),world_transform)
        heading=goal['yaw']+spawn['yaw']
        printable=[]
        for marker in scene['targets']:
            if marker['attached_to']!=name or error>=.65:
                continue
            delta=observer-np.array([marker['x'],marker['y']])
            distance=float(np.linalg.norm(delta))
            front=delta @ [-math.cos(marker['yaw']),-math.sin(marker['yaw'])]
            bearing=math.atan2(-delta[1],-delta[0])
            if .45<=distance<=1.5 and front/max(distance,.001)>.4 and math.cos(bearing-heading)>.866:
                printable.append(marker['id'])
        proposed.append(dict(geometrically_facing_ids=printable,time=round(elapsed,1),nearest=name,error_m=round(error,2),
                             matches_box=error<.65))
    return dict(input_maps=len(poses),suggestions=proposed)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('folder',type=Path)
    args=p.parse_args()
    print(json.dumps(replay(args.folder),indent=2))
