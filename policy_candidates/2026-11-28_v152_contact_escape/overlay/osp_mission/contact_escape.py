""".68: leave a close contact by turning in place and creeping, outside Nav2 and the collision monitor.

On 2026-11-27 the robot stopped next to desk legs three times. The nearest LiDAR return was
0.17-0.23 m away, so the footprint already overlapped an obstacle point. The policy then found no
reachable goal (inscribed start cost), and Nav2 spin was blocked. The collision monitor's approach
polygon also scaled every command to zero, the .59/.67 open-space Nav2 step included. Here the robot,
which is round, turns in place toward the most open direction and creeps a short distance there.
- The forward sector is checked on every scan.
- Create 3 hazards (bump, cliff, stall, wheel drop) stop the motion at once and stay authoritative.
"""
import math

import numpy as np

CONTACT_M = .32          # nearest return from the base centre that counts as a close contact
SECTOR = math.radians(15)
FORWARD_HALF = math.radians(35)
STOP_M = .25             # creep stops when a return ahead is closer than this (from base centre)
NEED_M = .70             # the chosen direction must be clear at least this far
STEP_M = .30
SPEED = .06
TURN_GAIN, TURN_MAX, ALIGNED = 1.2, .40, math.radians(8)
TURN_LIMIT_S, DRIVE_LIMIT_S, COOLDOWN_S = 12., 8., 20.


def base_points(ranges, angle_min, increment, range_min, range_max, laser_pose):
    """Scan returns as (x, y) in the base frame; laser_pose = (x, y, yaw) of the laser in base."""
    r = np.asarray(ranges, dtype=float)
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    a = angle_min + np.arange(len(r)) * increment + laser_pose[2]
    x = laser_pose[0] + r * np.cos(a)
    y = laser_pose[1] + r * np.sin(a)
    return np.stack((x[ok], y[ok]), axis=1)


def nearest(points):
    if not len(points):
        return math.inf
    return float(np.min(np.hypot(points[:, 0], points[:, 1])))


def open_heading(points, need=NEED_M):
    """Base-frame heading whose +-15 deg sector has the largest nearest distance (>= need)."""
    if not len(points):
        return None
    bearing = np.arctan2(points[:, 1], points[:, 0])
    dist = np.hypot(points[:, 0], points[:, 1])
    best = None
    for centre in np.linspace(-math.pi, math.pi, 72, endpoint=False):
        inside = np.abs(np.arctan2(np.sin(bearing - centre), np.cos(bearing - centre))) < SECTOR
        # No echo (glass, out of range) counts as only just open: a measured long range wins.
        free = float(dist[inside].min()) if inside.any() else NEED_M + .1
        if best is None or free > best[0]:
            best = (free, float(centre))
    return best[1] if best[0] >= need else None


def clear_ahead(points, stop=STOP_M):
    if not len(points):
        return True
    ahead = (points[:, 0] > 0) & (np.abs(np.arctan2(points[:, 1], points[:, 0])) < FORWARD_HALF)
    return not ahead.any() or float(np.hypot(points[ahead, 0], points[ahead, 1]).min()) >= stop


class ContactEscape:
    """Turn toward the open side, then creep STEP_M; every call returns (vx, wz) or None when done."""

    def __init__(self):
        self.phase = None
        self.last_end = -math.inf
        self.reason = ''

    def active(self):
        return self.phase is not None

    def start(self, now, points, pose):
        """pose = (x, y, yaw) in odom. Returns True if an escape started."""
        if self.phase is not None or now - self.last_end < COOLDOWN_S or nearest(points) > CONTACT_M:
            return False
        heading = open_heading(points)
        if heading is None:
            return False
        self.target = pose[2] + heading
        self.origin = pose[:2]
        self.phase, self.since, self.reason = 'turn', now, ''
        return True

    def end(self, now, reason):
        self.phase, self.reason, self.last_end = None, reason, now
        return None

    def step(self, now, points, pose, hazard):
        if self.phase is None:
            return None
        if hazard:
            return self.end(now, 'hazard')
        if self.phase == 'turn':
            error = math.remainder(self.target - pose[2], math.tau)
            if abs(error) <= ALIGNED:
                self.phase, self.since, self.origin = 'drive', now, pose[:2]
            elif now - self.since > TURN_LIMIT_S:
                return self.end(now, 'turn_timeout')
            else:
                return 0., float(np.clip(TURN_GAIN * error, -TURN_MAX, TURN_MAX))
        if math.dist(pose[:2], self.origin) >= STEP_M:
            return self.end(now, 'stepped')
        if now - self.since > DRIVE_LIMIT_S:
            return self.end(now, 'drive_timeout')
        if not clear_ahead(points):
            return self.end(now, 'blocked_ahead')
        error = math.remainder(self.target - pose[2], math.tau)
        return SPEED, float(np.clip(TURN_GAIN * error, -.2, .2))
