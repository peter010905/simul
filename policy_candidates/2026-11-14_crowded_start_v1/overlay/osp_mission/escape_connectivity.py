"""Reconnect a near-obstacle start to normal clearance without relaxing goals."""
from collections import deque
import math

import numpy as np
from scipy.ndimage import distance_transform_edt, label


def connect_start(data, resolution, start, safe, max_escape=None):
    """Connect a body-clear starting pocket to the nearest normal-clearance region.

    Normal destinations still require the original 0.32 m clearance. Unknown
    cells count as obstacles for this local exception. Cardinal steps prohibit
    corner cutting; the body radius includes cell extent and a 1 cm margin.
    """
    y, x = start
    if not (0 <= y < data.shape[0] and 0 <= x < data.shape[1]) or data[y, x] != 0:
        return safe
    groups, _ = label(safe)
    sizes = np.bincount(groups.ravel())
    start_group = groups[y, x]
    # A tiny inflated island is not a useful reachable region. A local wall
    # estimate can leave the robot on such an island even when a path exists.
    if start_group and sizes[start_group] * resolution**2 >= .5:
        return safe
    targets=sizes * resolution**2 >= .5
    targets[0]=False
    if start_group:targets[start_group]=False
    if not np.any(targets):return safe
    # An arbitrary 0.8 m limit excluded physically clear, longer exit routes.
    # Search observed free space only; an explicit bound remains available.
    clearance = distance_transform_edt(np.pad(data == 0,1,constant_values=False))[1:-1,1:-1] * resolution
    minimum = .189 + resolution / math.sqrt(2) + .01
    if clearance[y, x] < minimum:
        return safe
    queue = deque([(y, x)])
    parent = {(y, x): None}
    depth = {(y, x): 0}
    destination = None
    while queue:
        cell = queue.popleft()
        cy, cx = cell
        if safe[cell] and targets[groups[cell]]:
            destination = cell
            break
        if max_escape is not None and (depth[cell]+1) * resolution > max_escape:
            continue
        for ny, nx in ((cy-1,cx),(cy+1,cx),(cy,cx-1),(cy,cx+1)):
            nxt = (ny, nx)
            if not (0 <= ny < data.shape[0] and 0 <= nx < data.shape[1]) or nxt in parent:
                continue
            # Grid staircases need not have monotonically increasing clearance.
            # The physical body margin is required at EVERY step instead.
            if data[nxt] != 0 or clearance[nxt] < minimum:
                continue
            parent[nxt] = cell
            depth[nxt] = depth[cell]+1
            queue.append(nxt)
    if destination is None:
        return safe
    result = safe.copy()
    while destination is not None:
        result[destination] = True
        destination = parent[destination]
    return result
