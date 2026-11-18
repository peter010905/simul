"""Reconnect a near-obstacle start to normal clearance without relaxing goals."""
from collections import deque
import math

import numpy as np
from scipy.ndimage import distance_transform_edt


def connect_start(data, resolution, start, safe, max_escape=.8):
    """Add one short, known-free, nondecreasing-clearance escape corridor.

    Normal destinations still require the original 0.32 m clearance. Unknown
    cells count as obstacles for this local exception. Cardinal steps prohibit
    corner cutting; the body radius includes cell extent and a 1 cm margin.
    """
    y, x = start
    if not (0 <= y < data.shape[0] and 0 <= x < data.shape[1]) or data[y, x] != 0:
        return safe
    if safe[y, x]:
        return safe
    clearance = distance_transform_edt(data == 0) * resolution
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
        if safe[cell]:
            destination = cell
            break
        if (depth[cell]+1) * resolution > max_escape:
            continue
        for ny, nx in ((cy-1,cx),(cy+1,cx),(cy,cx-1),(cy,cx+1)):
            nxt = (ny, nx)
            if not (0 <= ny < data.shape[0] and 0 <= nx < data.shape[1]) or nxt in parent:
                continue
            if data[nxt] != 0 or clearance[nxt] < minimum or clearance[nxt] < clearance[cell]-1e-9:
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
