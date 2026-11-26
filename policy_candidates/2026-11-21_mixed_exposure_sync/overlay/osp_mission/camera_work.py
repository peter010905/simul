"""Bound exposure work and pick the newest frame whose exact-time TF is ready.

The queue owns no ROS executor or threads. Missing transforms must never block
the single-threaded detector that also receives TF and scans.
"""
from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Exposure:
    stamp: float
    message: object
    calibration: object


class ExposureQueue:
    def __init__(self, capacity=8, max_age=.4, future_tolerance=.1):
        self.capacity = capacity
        self.max_age = max_age
        self.future_tolerance = future_tolerance
        self.rows = deque()
        self.last_stamp = -math.inf
        self.last_clock = None
        self.counts = dict(received=0, replaced=0, expired=0, future=0,
                           duplicate=0, tf_waits=0, dispatched=0, clock_resets=0)

    def clock(self, now):
        if not math.isfinite(now):
            return False
        if self.last_clock is not None and now < self.last_clock-.5:
            self.rows.clear()
            self.last_stamp = -math.inf
            self.counts['clock_resets'] += 1
        self.last_clock = now
        return True

    def offer(self, message, calibration, stamp, now):
        self.counts['received'] += 1
        if not self.clock(now) or not math.isfinite(stamp) or stamp <= 0:
            return False
        if now-stamp < -self.future_tolerance:
            self.counts['future'] += 1
            return False
        if now-stamp > self.max_age:
            self.counts['expired'] += 1
            return False
        if stamp <= self.last_stamp:
            self.counts['duplicate'] += 1
            return False
        self.last_stamp = stamp
        if len(self.rows) == self.capacity:
            self.rows.popleft()
            self.counts['replaced'] += 1
        self.rows.append(Exposure(stamp, message, calibration))
        return True

    def take(self, now, ready):
        if not self.clock(now):
            return None
        while self.rows and now-self.rows[0].stamp > self.max_age:
            self.rows.popleft()
            self.counts['expired'] += 1
        # Keeping only the newest input would starve at high FPS when TF lags
        # more than one frame. Use the newest ready exposure within the bound.
        for index in range(len(self.rows)-1, -1, -1):
            row = self.rows[index]
            if not ready(row):
                continue
            for _ in range(index+1):
                self.rows.popleft()
            self.counts['replaced'] += index
            self.counts['dispatched'] += 1
            return row
        if self.rows:
            self.counts['tf_waits'] += 1
        return None


class PipelineTiming:
    def __init__(self):
        self.samples = deque(maxlen=256)

    def record(self, stamp, now, elapsed):
        self.samples.append((float(stamp), float(now-stamp), float(elapsed)))

    def report(self):
        if not self.samples:
            return dict(samples=0)
        rows = list(self.samples)
        percentile = lambda values: sorted(values)[min(len(values)-1, math.ceil(.95*len(values))-1)]
        elapsed = rows[-1][0]-rows[0][0]
        return dict(samples=len(rows), capture_age_p95_s=percentile([r[1] for r in rows]),
                    callback_wall_p95_s=percentile([r[2] for r in rows]),
                    dispatched_fps_ros=(len(rows)-1)/elapsed if elapsed > 0 else None)
