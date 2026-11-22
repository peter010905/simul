"""Rank reachable box corners by unseen faces in view and local travel cost."""
import math
from collections import deque

import numpy as np


def walking_distance(mask, start, resolution, limit_m=4.5):
    """Four-neighbour walking lengths on the same clearance mask as the planner."""
    max_steps = math.floor(limit_m / resolution)
    steps = {start: 0}
    todo = deque([start])
    height,width = mask.shape
    while todo:
        row,col = todo.popleft()
        step = steps[row,col]+1
        if step > max_steps:
            continue
        for r,c in ((row-1,col),(row+1,col),(row,col-1),(row,col+1)):
            if 0<=r<height and 0<=c<width and mask[r,c] and (r,c) not in steps:
                steps[r,c] = step
                todo.append((r,c))
    return steps


def rate_corner(grid, view, face, adjacent, side, odom_yaw, seen, heading, path_m, half_fov):
    """Choose one aiming point; credit both faces only if both fit the camera."""
    look = face
    gain = 1
    angle = math.atan2(side[1],side[0]) + odom_yaw
    other_unseen = all(math.cos(angle-a) <= math.cos(math.radians(35)) for a in seen)
    if other_unseen and grid.line_clear(view,adjacent+.12*side):
        midpoint = (face+adjacent)/2
        aim = midpoint-view
        yaw = math.atan2(aim[1],aim[0])
        def readable(point):
            ray = point-view
            bearing = math.atan2(ray[1],ray[0])
            return np.linalg.norm(ray)<=1.45 and math.cos(bearing-yaw)>=math.cos(half_fov)
        if readable(face) and readable(adjacent):
            look = midpoint
            gain = 2
    ray = look-view
    yaw = math.atan2(ray[1],ray[0])
    turn = abs(math.remainder(yaw-heading,2*math.pi))
    # Distances are a ranking proxy. Fixed settling and turning are expressed
    # as equivalent walking distances, not predictions of elapsed seconds.
    score = gain / (.65 + path_m + .22*turn)
    return score, look, yaw, gain
