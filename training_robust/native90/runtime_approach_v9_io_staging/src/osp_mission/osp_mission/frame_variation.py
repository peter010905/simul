"""Rigid map-frame changes for checking coordinate bias in the 2D benchmark."""

import copy
import math

import numpy as np

from osp_mission.policy import wrap


def rotate_scene_frame(config, occupied, resolution, quarter_turns):
    """Rotate representation clockwise, preserving the physical scene exactly."""
    if quarter_turns not in (0,1,2,3):
        raise ValueError('Map-frame rotation must be 0, 1, 2 or 3 quarter turns')
    if not quarter_turns:
        return config,occupied
    result=copy.deepcopy(config)
    grid=occupied
    for _ in range(quarter_turns):
        width=grid.shape[1]*resolution
        x,y,yaw=result['start']
        result['start']=[y,width-x,wrap(yaw-math.pi/2)]
        for obj in result.get('targets',[])+result.get('obstacles',[]):
            x,y=obj['x'],obj['y'];obj['x']=y;obj['y']=width-x
            if 'yaw' in obj:obj['yaw']=wrap(obj['yaw']-math.pi/2)
        grid=np.ascontiguousarray(np.rot90(grid))
    return result,grid
