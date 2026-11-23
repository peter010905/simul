"""A bounded closer look when a native decoded print has no measured depth."""
from collections import deque
import math

import numpy as np

from osp_mission.r32_drive import in_map


class BearingAdvance:
    def __init__(self):
        self.windows = {}
        self.supported = {}
        self.attempts = []

    def observe(self, cue, now):
        ident = cue.get('id')
        if type(ident) is not int or not 0 <= ident < 1024:
            return
        stamp = float(cue['stamp'])
        origin = tuple(map(float, cue['origin_odom']))
        angle, pixels = float(cue['yaw_odom']), float(cue['pixels'])
        if (len(origin) != 2 or not all(map(math.isfinite, (*origin, stamp, now, angle, pixels)))
                or not -.1 <= now - stamp <= .65 or pixels < 10.):
            return
        row = (stamp, origin, angle, pixels)
        window = self.windows.setdefault(ident, deque(maxlen=3))
        if window and stamp <= window[-1][0]:
            return
        window.append(row)
        if len(window) < 3 or not .18 <= stamp - window[0][0] <= .85:
            return
        if any(math.dist(p, origin) > .25 or abs(math.remainder(a-angle, math.tau)) > .05
               or not .75 <= size / pixels <= 1.33 for _, p, a, size in window):
            return
        self.supported[ident] = row

    def choose(self, grid, cost, safe, nav, distances, pose, transform, known, now, remaining, veto):
        if remaining < 12. or len(self.attempts) >= 4 or not self.supported:
            return None
        best = None
        for ident, (stamp, origin, angle, pixels) in list(self.supported.items()):
            if ident in known or not 0 <= now-stamp <= 30.:
                del self.supported[ident]
                continue
            attempts = [a for a in self.attempts if a[0] == ident]
            if len(attempts) >= 2:
                continue
            if attempts:
                _, prior_stamp, prior_origin, prior_size = attempts[-1]
                if stamp <= prior_stamp or math.dist(prior_origin, origin) < .65 or pixels < 1.12 * prior_size:
                    continue
            observer = np.asarray(in_map(('odom', *origin), transform))
            bearing = angle - transform[2]
            axis = np.array([math.cos(bearing), math.sin(bearing)])
            tangent = np.array([-axis[1], axis[0]])
            projected = float((np.asarray(pose[:2]) - observer) @ axis)
            # The ray supplies direction only. No depth is assigned to the marker.
            # Do not drive back to the old camera position, or steer around a
            # corner to an endpoint on a different, unobserved ray.
            for forward in (1.4, 1., .65):
                progress = forward - projected
                if progress < .40:
                    continue
                for lateral in (0., -.25, .25):
                    point = observer + forward * axis + lateral * tangent
                    cell = grid.index(*point)
                    nc = cost.index(*point)
                    if (not grid.inside(*cell) or not safe[cell] or not cost.inside(*nc) or not nav[nc]
                            or not grid.line_clear(observer, point)):
                        continue
                    walk = float(distances[cell])
                    if not .3 <= walk <= min(2.4, .20 * (remaining-8.)):
                        continue
                    sweep = math.radians(24)
                    goal = dict(x=float(point[0]), y=float(point[1]), yaw=bearing-sweep/2,
                                r32_kind='named_bearing_relook', face_inspection=True, stationary=False,
                                scan_angle=sweep, distance=walk, features=np.array([0., 0., walk, 0., 0.]),
                                bearing_advance_id=ident, bearing_stamp=stamp, bearing_origin=origin,
                                bearing_pixels=pixels, bearing_progress_m=progress)
                    if veto(goal):
                        continue
                    rank = progress / (1.+walk+abs(lateral))
                    if best is None or rank > best[0]:
                        best = rank, goal
        return best[1] if best else None

    def dispatched(self, goal):
        if 'bearing_advance_id' in goal:
            self.attempts.append((goal['bearing_advance_id'], goal['bearing_stamp'],
                                  goal['bearing_origin'], goal['bearing_pixels']))
