"""Keep a time-bounded multi-view history across camera rates and moving views."""
from collections import deque
import math
import numpy as np
from osp_mission.marker_relook import VisualRange, Sight


class CameraVisualRange(VisualRange):
    def __init__(self):
        super().__init__()
        self.last_clock = -math.inf
        self.views = {}

    def observe(self, ident, stamp, origin, direction, pixels):
        if not np.isfinite([stamp, pixels, *origin, *direction]).all():
            return None
        if stamp < self.last_clock-.5:
            self.history.clear()
            self.views.clear()
        self.last_clock = max(stamp, self.last_clock) if stamp >= self.last_clock-.5 else stamp
        self.history = {key: rows for key, rows in self.history.items()
                        if 0 <= stamp-rows[-1].stamp <= 60}
        self.views = {key: views for key, views in self.views.items()
                      if views and 0 <= stamp-views[-1][-1].stamp <= 60}
        rows = self.history.setdefault(ident, deque(maxlen=16))
        # Keep dense samples only for the local window, then retain triplet
        # keyframes at <=5 Hz. Searching history must not become quadratic.
        if rows and stamp-rows[-1].stamp < .08-1e-9:
            return None
        while rows and stamp-rows[0].stamp > 1.:
            rows.popleft()
        rows.append(Sight(stamp, np.array(origin, copy=True), np.array(direction, copy=True), pixels))
        current = self.window(rows, stamp, origin)
        if len(current) < 3:
            return None
        views = self.views.setdefault(ident, deque(maxlen=320))
        while views and stamp-views[0][-1].stamp > 60:
            views.popleft()
        if not views or stamp-views[-1][-1].stamp >= .2-1e-9:
            views.append(current)
        for earlier in reversed(views):
            seed=earlier[-1]
            if not 1 <= stamp-seed.stamp <= 60 or np.linalg.norm(seed.origin-origin) < .35:
                continue
            angle=math.degrees(math.acos(np.clip(earlier[1].direction@current[1].direction,-1,1)))
            if not 12 <= angle <= 145:
                continue
            try:
                point=self.intersection(earlier+current)
            except np.linalg.LinAlgError:
                point=None
            # A recent low-parallax or inconsistent pair must not prevent a
            # valid older viewpoint from being evaluated.
            if point is not None:
                return point
        return None

    @staticmethod
    def window(rows, end, origin):
        # Positions are retained separately and used by the ray intersection;
        # this is not a stationary-camera approximation. At 0.25 m/s, three
        # 5-FPS views span 0.10 m and were wrongly excluded by the old 0.09 m.
        window = [r for r in rows if 0 <= end-r.stamp <= .55
                  and np.linalg.norm(r.origin-origin) <= .14]
        if len(window) < 3 or window[-1].stamp-window[0].stamp < .18:
            return []
        return [window[0], window[len(window)//2], window[-1]]
