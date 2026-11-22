"""Offline test of a bearing-only approach hint, independent of ID registration."""
import math

import numpy as np
from osp_mission.camera_multiview import CameraVisualRange


class DistantHint(CameraVisualRange):
    """Triangulate native decoded prints only to select a closer look."""

    @staticmethod
    def intersection(views):
        rays = np.array([r.direction for r in views])
        origins = np.array([r.origin for r in views])
        cosine = np.clip(rays[1] @ rays[4], -1., 1.)
        angle = math.degrees(math.acos(cosine))
        if not 15 <= angle <= 120:
            return None
        projectors = np.eye(3) - rays[:, :, None] * rays[:, None, :]
        point = np.linalg.solve(projectors.sum(axis=0),
                                (projectors @ origins[:, :, None]).sum(axis=0)).ravel()
        delta = point - origins
        depths = (delta * rays).sum(axis=1)
        if depths.min() < .3 or depths.max() > 8. or depths[-1] > 5.5:
            return None
        errors = np.linalg.norm(delta - depths[:, None]*rays, axis=1)
        if errors.max() > .12 or np.max(np.arctan2(errors, depths)) > math.radians(3):
            return None
        sizes = depths * np.array([r.pixels for r in views])
        if sizes.max() > 1.5*sizes.min():
            return None
        return point

    def __init__(self):
        super().__init__()
        self.estimates = {}

    def observe(self, ident, stamp, origin, direction, pixels):
        if stamp < self.last_clock-.5:
            self.estimates.clear()
        point = super().observe(ident, stamp, origin, direction, pixels)
        self.estimates = {key: sample for key, sample in self.estimates.items()
                          if 0 <= stamp - sample[-1][0] <= 1.}
        if point is None:
            return None
        old = self.estimates.get(ident, [])
        if old and stamp <= old[-1][0]:
            return None
        samples = [(t,p) for t,p in old if stamp-t <= 1.]
        if any(np.linalg.norm(point-p) > .20 for _,p in samples):
            samples = []
        samples.append((stamp, point))
        self.estimates[ident] = samples[-3:]
        if len(samples) < 3 or stamp - samples[-3][0] < .18:
            return None
        return np.median([p for _,p in samples[-3:]], axis=0)
