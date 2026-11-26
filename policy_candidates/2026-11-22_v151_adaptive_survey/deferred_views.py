"""Allow one extra inspection when an unobserved face gains a new approach."""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np
from osp_mission.box_views import odom_point


@dataclass
class Attempt:
    kind: str
    anchor: tuple
    yaw: float
    time: float
    availability: int
    viewpoint: tuple


class DeferredViews:
    def __init__(self):
        self.history = deque(maxlen=200)
        self.geometry = None

    def update_geometry(self, grid, cost, nav, distances, clearance, transform):
        self.geometry = grid, cost, nav, distances, clearance, transform

    def available(self, anchor, yaw):
        grid, cost, nav, distances, clearance, transform = self.geometry
        tx, ty, rotation = transform
        c, s = math.cos(rotation), math.sin(rotation)
        x, y = anchor[0] - tx, anchor[1] - ty
        face = np.array((c*x + s*y, -s*x + c*y))
        normal = np.array((math.cos(yaw - rotation), math.sin(yaw - rotation)))
        tangent = np.array((-normal[1], normal[0]))
        # Sample again at the original physical anchor even if SLAM resizes.
        mask = 0
        for i, (stand, lateral) in enumerate((d, l) for d in (.55, .9, 1.25)
                                              for l in (-.30, 0., .30)):
            point = face + stand * normal + lateral * tangent
            cell, nav_cell = grid.index(*point), cost.index(*point)
            if (grid.inside(*cell) and cost.inside(*nav_cell) and nav[nav_cell]
                    and clearance[cell] >= .32 and math.isfinite(distances[cell])
                    and grid.line_clear(point, face + .06 * normal)):
                mask |= 1 << i
        return mask

    def view_filter(self, kind, anchor, yaw, radius, attempts, elapsed, veto):
        if attempts and elapsed - attempts[-1] < 45.:
            return None
        if len(attempts) < 2:
            return veto
        if len(attempts) >= 3 or self.geometry is None:
            return None
        matches = [row for row in self.history if row.kind == kind and
                   math.dist(row.anchor, anchor) < radius and math.cos(row.yaw - yaw) > .70]
        if len(matches) != 2:
            return None
        last = matches[-1]
        # Require at least two newly feasible samples since the last attempt.
        # Never reopen merely because a clock expired or one noisy cell flipped.
        if (self.available(last.anchor, last.yaw) & ~last.availability).bit_count() < 2:
            return None
        transform = self.geometry[-1]

        def retry_veto(goal):
            position = odom_point((goal['x'], goal['y']), transform)
            if veto(goal) or any(math.dist(row.viewpoint, position) < .32 for row in matches):
                return True
            goal['reopened_face'] = True
            return False
        return retry_veto

    def dispatched(self, phase, goal, elapsed):
        if self.geometry is None or 'surface_odom' not in goal:
            return
        anchor, yaw = tuple(goal['surface_odom']), float(goal['normal_odom'])
        pos = tuple(odom_point((goal['x'], goal['y']), self.geometry[-1]))
        self.history.append(Attempt(phase, anchor, yaw, elapsed, self.available(anchor, yaw), pos))
