"""Plan box approaches through the intersection of current safety masks."""
import math

import numpy as np

from osp_mission._r32_navigation_core import travel_distances
from osp_mission.adaptive_faces import indices


def box_routes(grid, cost, nav, safe, pose, transform, local=None):
    joint = np.zeros_like(safe)
    rr, cc = np.where(safe)
    ox, oy, angle = grid.origin
    c, s = math.cos(angle), math.sin(angle)
    u, v = (cc+.5)*grid.resolution, (rr+.5)*grid.resolution
    points = np.column_stack((ox+c*u-s*v, oy+s*u+c*v))
    r, col = indices(cost, points)
    inside = (r >= 0) & (col >= 0) & (r < nav.shape[0]) & (col < nav.shape[1])
    keep = np.zeros(len(rr), dtype=bool)
    keep[inside] = nav[r[inside], col[inside]]
    if local is not None:
        frame, local_grid = local
        if frame == 'odom':
            tx, ty, yaw = transform
            c, s = math.cos(yaw), math.sin(yaw)
            points = points @ np.array([[c, s], [-s, c]]) + (tx, ty)
        r, col = indices(local_grid, points)
        rows, cols = local_grid.cells.shape
        inside = (r >= 0) & (col >= 0) & (r < rows) & (col < cols)
        # The rolling local costmap already accounts for the robot footprint.
        keep[inside] &= local_grid.cells[r[inside], col[inside]] < 99
    joint[rr, cc] = keep
    start = grid.index(*pose[:2])
    if not grid.inside(*start) or not joint[start]:
        return joint, np.full(safe.shape, np.inf)
    return joint, travel_distances(joint, *start, grid.resolution)
