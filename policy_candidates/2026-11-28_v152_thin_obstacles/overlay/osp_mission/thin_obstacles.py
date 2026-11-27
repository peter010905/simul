""".69: keep thin mapped obstacles (desk and chair legs) in the global costmap.

Legs 2-4 cm across are hit by only some LiDAR beams. The scan source of the global obstacle layer
marks and clears them in turn, so a planned path could run past a leg at 0.2 m, and Regulated Pure
Pursuit follows a path without avoiding. On 2026-11-27 this left the robot against single legs three
times. Small occupied components of the SLAM map are re-published as marking-only points on the
existing `mission/known_obstacles` source (clearing: false). The planner then keeps them and their
inflation even when a scan misses them.
"""
import numpy as np
from scipy import ndimage

MAX_EXTENT_M = .30     # boxes (>= 0.46 m) and walls are not included
MIN_CELLS = 2


def thin_obstacle_points(data, resolution, origin, z=.10):
    """Cell centres (map frame) of occupied components no larger than MAX_EXTENT_M."""
    occupied = np.asarray(data) >= 65
    labels, count = ndimage.label(occupied, structure=np.ones((3, 3)))
    if not count:
        return np.zeros((0, 3), np.float32)
    keep = np.zeros(count + 1, bool)
    for ident, region in enumerate(ndimage.find_objects(labels), 1):
        if region is None:
            continue
        extent = max(region[0].stop - region[0].start, region[1].stop - region[1].start) * resolution
        keep[ident] = extent <= MAX_EXTENT_M and np.count_nonzero(labels[region] == ident) >= MIN_CELLS
    rows, cols = np.nonzero(keep[labels])
    return np.stack((origin[0] + (cols + .5) * resolution, origin[1] + (rows + .5) * resolution,
                     np.full(len(rows), z)), axis=1).astype(np.float32)
