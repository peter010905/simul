"""Conservative static-scene evidence, not semantic recognition or collision clearing."""
import numpy as np


class Consensus:
    def __init__(self, occupied, resolution, minimum_hits=3, minimum_span_s=6., max_free_ratio=.25):
        self.occupied = np.asarray(occupied, dtype=bool)
        self.resolution = float(resolution)
        self.minimum_hits = minimum_hits
        self.minimum_span_s = minimum_span_s
        self.max_free_ratio = max_free_ratio
        self.hits = np.zeros(self.occupied.shape, dtype=np.uint16)
        self.free = np.zeros(self.occupied.shape, dtype=np.uint16)
        self.first = np.full(self.occupied.shape, np.inf)
        self.last = np.full(self.occupied.shape, -np.inf)
        self.views = []

    def add(self, stamp, pose, hit, free):
        """Call with time-aligned raw LiDAR evidence in one run's map frame.

        A repeated scan from the same pose is not an independent viewpoint.
        The image/map publisher's repeated snapshots are never counted as hits.
        """
        pose = np.asarray(pose)
        if self.views:
            t, previous = self.views[-1]
            turn = abs(np.arctan2(np.sin(pose[2]-previous[2]), np.cos(pose[2]-previous[2])))
            if stamp-t < 2. or (np.linalg.norm(pose[:2]-previous[:2]) < .25 and turn < np.deg2rad(15)):
                return False
        hit = np.asarray(hit, dtype=bool) & self.occupied
        free = np.asarray(free, dtype=bool) & self.occupied & ~hit
        self.hits[hit] += 1
        self.free[free] += 1
        self.first[hit] = np.minimum(self.first[hit], stamp)
        self.last[hit] = np.maximum(self.last[hit], stamp)
        self.views.append((float(stamp), pose.copy()))
        return True

    def mask(self):
        span = np.zeros(self.hits.shape)
        seen = self.hits > 0
        span[seen] = self.last[seen] - self.first[seen]
        return (self.occupied & (self.hits >= self.minimum_hits) & (span >= self.minimum_span_s)
                & (self.free <= self.max_free_ratio * np.maximum(self.hits, 1)))


def scan_evidence(points_world, origin, yaw, ranges, angle_min, angle_increment,
                  range_min, range_max, slam_range, tolerance=.12):
    """Classify final-map occupied-cell centers using actual rays, not inferred emptiness.

    Invalid/out-of-FOV/out-of-SLAM-range rays do not become free observations.
    Unknown and never-reobserved geometry remains unverified.
    """
    points = np.asarray(points_world)
    d = points - np.asarray(origin)
    distance = np.linalg.norm(d, axis=1)
    angle = np.arctan2(d[:, 1], d[:, 0]) - yaw
    angle = np.arctan2(np.sin(angle), np.cos(angle))
    indices = np.rint((angle-angle_min)/angle_increment).astype(int)
    valid_index = (indices >= 0) & (indices < len(ranges))
    measured = np.asarray(ranges)[np.clip(indices, 0, len(ranges)-1)]
    valid = (valid_index & np.isfinite(measured) & (measured >= range_min)
             & (measured <= range_max) & (distance >= range_min) & (distance <= slam_range))
    hit = valid & (abs(measured-distance) <= tolerance)
    free = valid & (measured-distance > tolerance * 2)
    return hit, free
