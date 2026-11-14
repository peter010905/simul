"""Mission-long odometry memory for redundant exploration viewpoints.

This never bans travel through a corridor. Marker follow-up/approach runs after
this filter and can override it. New viewing directions and frontier gain stay
eligible. Odometry anchors survive SLAM map-frame corrections.
"""
import math

import numpy as np


def transform_pose(x, y, yaw, transform):
    tx, ty, angle = transform
    c, s = math.cos(angle), math.sin(angle)
    return tx+c*x-s*y, ty+s*x+c*y, (yaw+angle+math.pi) % (2*math.pi)-math.pi


class ExplorationMemory:
    def __init__(self):
        self.cells = {}
        self.views = []
        self.last_pose = None
        self.path_distance_m = 0.
        self.revisited_distance_m = 0.
        self.pending = None
        self.suppressed = 0
        self.failures = []
        self.retry_after = 0.

    def observe(self, x, y, stamp):
        if not np.isfinite([x, y, stamp]).all():
            return
        cell = (math.floor(x/.6), math.floor(y/.6))
        if self.last_pose is not None:
            distance = math.hypot(x-self.last_pose[0], y-self.last_pose[1])
            if distance < .20:
                return
            if distance > 1.5:
                # Do not count localization jumps as physically traveled distance.
                self.last_pose = (x, y)
                return
            self.path_distance_m += distance
            if cell in self.cells and stamp-self.cells[cell]['last_exit'] > 12:
                self.revisited_distance_m += distance
        previous = self.cells.get(cell, {'entries': 0, 'last_exit': -100})
        if self.last_pose is None or cell != (math.floor(self.last_pose[0]/.6), math.floor(self.last_pose[1]/.6)):
            previous['entries'] += 1
        previous['last_exit'] = stamp
        self.cells[cell] = previous
        self.last_pose = (x, y)

    def filter(self, candidates, transform, stamp):
        kept = []
        self.suppressed = 0
        retry_times = []
        for original in candidates:
            candidate = dict(original)
            candidate['features'] = np.array(original['features'], dtype=float).copy()
            x, y, yaw = transform_pose(candidate['x'], candidate['y'], candidate['yaw'], transform)
            informative_frontier = candidate['features'][1] >= .15
            duplicates = [v for v in self.views if math.hypot(x-v['x'], y-v['y']) < .8
                and abs((yaw-v['yaw']+math.pi) % (2*math.pi)-math.pi) < math.pi/4
                and 0 <= stamp-v['stamp'] < 150]
            # A large but UNCHANGED frontier estimate is not progress. Reopen
            # only when new sensor evidence raises its gain, or its TTL expires.
            duplicate = any(candidate['features'][1] <= v.get('gain', 0.)+.15 for v in duplicates)
            failed = [v for v in self.failures if math.hypot(x-v['x'], y-v['y']) < .65
                      and stamp < v['retry_at']]
            local = sum(min(3, record['entries'])*math.exp(-((x-(cx+.5)*.6)**2+(y-(cy+.5)*.6)**2)/.6)
                        for (cx, cy), record in self.cells.items())
            candidate['features'][4] += min(4., local)*(.15 if informative_frontier else 1.)
            if duplicate or failed:
                self.suppressed += 1
                retry_times += [v['stamp']+150 for v in duplicates]
                retry_times += [v['retry_at'] for v in failed]
                continue
            kept.append(candidate)
        # Never undo the entire filter when it removes all goals. The mission
        # waits/replans on new maps; expiration permits bounded later retries.
        self.retry_after = min(retry_times, default=stamp)
        return kept

    def selected(self, candidate, transform):
        self.pending = None
        if candidate is None or 'approach_id' in candidate or 'focus_id' in candidate:
            return
        x, y, yaw = transform_pose(candidate['x'], candidate['y'], candidate['yaw'], transform)
        self.pending = dict(x=x, y=y, yaw=yaw, gain=float(candidate['features'][1]))

    def completed(self, success, stamp):
        if success and self.pending is not None:
            self.views.append(dict(self.pending, stamp=stamp))
            self.failures = [v for v in self.failures if math.hypot(
                v['x']-self.pending['x'], v['y']-self.pending['y']) >= .65]
        elif self.pending is not None:
            recent = [v for v in self.failures if stamp-v['stamp'] < 300 and math.hypot(
                v['x']-self.pending['x'], v['y']-self.pending['y']) < .65]
            attempts = 1+max((v['attempts'] for v in recent), default=0)
            self.failures.append(dict(self.pending, stamp=stamp, attempts=attempts,
                                     retry_at=stamp+min(90., 15.*2**min(attempts-1, 3))))
        self.views = [v for v in self.views if stamp-v['stamp'] < 300]
        self.failures = [v for v in self.failures if stamp-v['stamp'] < 300]
        self.pending = None

    def report(self):
        return dict(version=2, visited_cells=len(self.cells), completed_viewpoints=len(self.views),
                    failed_viewpoints=len(self.failures), retry_after=self.retry_after,
                    suppressed_redundant_viewpoints=self.suppressed,
                    path_distance_m=self.path_distance_m,
                    revisited_distance_m=self.revisited_distance_m,
                    source='odom + observed SLAM/TF only; revisits include necessary transit')
