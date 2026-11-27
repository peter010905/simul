"""Admit short Nav2 recovery goals using the local swept body and current action."""
from collections import deque
import math

import numpy as np


class ReverseLimit:
    """Rearm a limited retreat only after hardware release and forward odometry."""

    def __init__(self):
        self.latched = False
        self.asserted = False
        self.last_report = -math.inf
        self.clear_since = None
        self.anchor = None

    def report(self, asserted, now):
        self.last_report = now
        if asserted:
            if not self.asserted:
                self.anchor = None
            self.latched = True
            self.clear_since = None
        elif self.asserted:
            self.clear_since = now
        self.asserted = asserted

    def odometry(self, pose, now):
        if not self.latched or not all(math.isfinite(v) for v in pose):
            return
        if self.anchor is None:
            self.anchor = pose
            return
        x, y, yaw = self.anchor
        forward = (pose[0] - x)*math.cos(yaw) + (pose[1] - y)*math.sin(yaw)
        if self.asserted:
            # The base can move forward while its prior reverse limit is still
            # reported. Measure recovery from the deepest point of that retreat.
            if forward < 0.:
                self.anchor = (pose[0], pose[1], yaw)
            return
        if (self.clear_since is not None and now - self.clear_since >= .5
                and 0 <= now - self.last_report <= 1.0 and forward >= .06):
            self.latched = False


class DriveProgress:
    """Keep a continuous, per-action window of odometric translation and turning."""

    def __init__(self):
        self.samples = deque()

    def reset(self):
        self.samples.clear()

    def stalled(self, now, pose):
        if pose is None or not all(math.isfinite(x) for x in (now, *pose)):
            self.reset()
            return False
        if self.samples and not 0 < now - self.samples[-1][0] <= 1.5:
            self.reset()
        self.samples.append((now, pose))
        while len(self.samples) > 1 and self.samples[1][0] <= now - 10:
            self.samples.popleft()
        first_time, first = self.samples[0]
        if now - first_time < 10:
            return False
        moved = any(math.dist(p[:2], first[:2]) >= .10 for _, p in self.samples)
        # A slow but productive initial alignment gets the same bounded allowance
        # as TransitFlow. Genuine sustained translation stalls are still cancelled.
        recent = next(p for t, p in self.samples if t >= now - 2.5)
        turning = abs(math.remainder(pose[2] - recent[2], math.tau)) >= .15
        return not moved and not turning


def escape_headings(scan, sensor_pose, footprint_radius, step=.60, need=.90, margin=.05):
    """Rank feasible headings. A missing echo cannot win by claiming maximum range."""
    ranges = np.asarray(scan.ranges, dtype=float)
    if (len(ranges) < 9 or not np.isfinite((*sensor_pose, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max, footprint_radius, step, need, margin)).all()
            or scan.range_max <= need or abs(scan.angle_increment) < 1e-8):
        return []
    sx, sy, syaw = sensor_pose
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment + syaw
    finite = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges <= scan.range_max)
    x = sx + ranges[finite] * np.cos(angles[finite])
    y = sy + ranges[finite] * np.sin(angles[finite])
    if not len(x):
        return []
    radius = footprint_radius + margin
    rotation_clear = np.min(np.hypot(x, y)) > radius
    candidates = []
    for degrees in range(-180, 180, 10):
        if degrees and not rotation_clear:
            continue
        a = math.radians(degrees)
        c, s = math.cos(a), math.sin(a)
        forward = x*c + y*s
        lateral = -x*s + y*c
        # Distance to the full swept disk, including the endpoint and both flanks.
        # Returns behind the start get farther away during a straight departure.
        before_end = np.clip(forward, 0., step)
        clearance = np.hypot(forward - before_end, lateral)
        if np.any((forward >= 0) & (clearance <= radius)):
            continue
        # Require measured depth across the forward sector. Sparse/no-return data
        # cannot certify an open corridor. The independently checked Nav2 goal
        # remains subject to the normal footprint and collision monitor.
        rel = np.arctan2(lateral, forward)
        half = math.radians(15)
        central = (np.abs(rel) <= half) & (np.hypot(x, y) >= need)
        supports = np.sort(rel[central])
        if len(supports) < 3 or np.diff(np.r_[-half, supports, half]).max() > math.radians(12):
            continue
        if np.any((np.abs(rel) <= half) & (np.hypot(x, y) < need)):
            continue
        ahead = clearance[forward >= 0]
        useful_clearance = min(float(ahead.min()) if len(ahead) else need, need)
        candidates.append((abs(a), -useful_clearance, a))
    return [angle for _, _, angle in sorted(candidates)]
