"""Collect readable, identified tags across separate camera positions."""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class Sight:
    stamp: float
    origin: np.ndarray
    direction: np.ndarray
    pixels: float


class VisualRange:
    def __init__(self):
        self.history = {}

    def observe(self, ident, stamp, origin, direction, pixels):
        # Limit memory to recent detections, and weight both visits equally.
        self.history = {key: rows for key, rows in self.history.items()
                        if stamp - rows[-1].stamp <= 60}
        rows = self.history.setdefault(ident, deque(maxlen=180))
        if rows and stamp <= rows[-1].stamp:
            return None
        rows.append(Sight(stamp, origin, direction, pixels))
        current = self.window(rows, stamp, origin)
        if len(current) < 3:
            return None
        for seed in reversed(rows):
            if not 1 <= stamp - seed.stamp <= 60 or np.linalg.norm(seed.origin - origin) < .35:
                continue
            earlier = self.window(rows, seed.stamp, seed.origin)
            if len(earlier) >= 3:
                return self.intersection(earlier + current)
        return None

    @staticmethod
    def window(rows, end, origin):
        window = [r for r in rows if 0 <= end-r.stamp <= .55
                  and np.linalg.norm(r.origin-origin) <= .09]
        if len(window) < 3 or window[-1].stamp-window[0].stamp < .18:
            return []
        return [window[0], window[len(window)//2], window[-1]]

    @staticmethod
    def intersection(views):
        rays = np.array([r.direction for r in views])
        angle = math.degrees(math.acos(np.clip(rays[1] @ rays[4], -1, 1)))
        if not 12 <= angle <= 145:
            return None
        origins = np.array([r.origin for r in views])
        projectors = np.eye(3) - rays[:, :, None] * rays[:, None, :]
        point = np.linalg.solve(projectors.sum(axis=0),
                                (projectors @ origins[:, :, None]).sum(axis=0)).ravel()
        delta = point - origins
        depth = (delta * rays).sum(axis=1)
        residual = np.linalg.norm(delta - rays * depth[:, None], axis=1)
        if min(depth) < .3 or max(depth) > 4 or max(residual) > .12:
            return None
        if max(np.arctan2(residual, depth)) > math.radians(3):
            return None
        # A second view of the same physical print should have compatible scale.
        sizes = depth * [r.pixels for r in views]
        if max(sizes) > 1.5 * min(sizes):
            return None
        return point


def sight_in_odom(transform, ray):
    p, q = transform.translation, transform.rotation
    orientation = Rotation.from_quat([q.x, q.y, q.z, q.w])
    direction = orientation.apply(ray / np.linalg.norm(ray))
    return np.array([p.x, p.y, p.z]), direction
