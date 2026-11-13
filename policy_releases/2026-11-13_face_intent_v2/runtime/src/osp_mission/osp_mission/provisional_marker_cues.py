"""Stable small image detections may request inspection, never ID recognition."""
import numpy as np


class ProvisionalMarkerCues:
    def __init__(self):
        self.history = {}
        self.last_published = {}

    def observe(self, ident, point, stamp, pixels):
        point = np.asarray(point, dtype=float)
        if point.shape != (3,) or not np.isfinite([*point, stamp, pixels]).all():
            return False
        if not 10. <= pixels < 18.:
            return False
        old = self.history.get(ident)
        if old is not None and stamp <= old['stamp']:
            return False
        consistent = (old is not None and stamp-old['stamp'] <= 1.
                      and np.linalg.norm(point-old['point']) <= .25)
        count = old['count']+1 if consistent else 1
        self.history[ident] = dict(point=point, stamp=stamp, count=count)
        if count < 3 or stamp-self.last_published.get(ident, -float('inf')) < 2.:
            return False
        self.last_published[ident] = stamp
        return True
