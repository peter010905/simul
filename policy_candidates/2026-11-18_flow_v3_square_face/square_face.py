"""Anonymous printed-border tracking and decision-time priority over weak surfaces."""
from dataclasses import dataclass
from collections import Counter
import math

import cv2
import numpy as np

from surface_memory import SurfaceMemory
from visual_cue import paper_candidate


def borders(image):
    """Propose outlines from contrast alone, without consulting any marker dictionary."""
    _, otsu = cv2.threshold(image, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    adaptive = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY_INV, 15, 5)
    found = []
    for mask in (otsu, adaptive):
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if not 65 <= area <= image.size * .4:
                continue
            perimeter = cv2.arcLength(contour, True)
            quad = cv2.approxPolyDP(contour, .035 * perimeter, True)
            if len(quad) != 4 or not cv2.isContourConvex(quad):
                continue
            match = paper_candidate(image, quad.astype(np.float32))
            if match and all(math.dist(match[:2], old[:2]) > 3 for old in found):
                found.append(match)
    return found


def associate(u, size, samples, camera, cal):
    """A laser-supported surface at the horizontal bearing, with a depth-edge veto."""
    near = []
    for point, normal, _ in samples:
        dx, dy = point - camera[:2]
        bearing = math.atan2(dy, dx) - camera[2]
        pixel = cal.k[2] - cal.k[0] * math.tan(bearing)
        error = abs(pixel-u)
        if error <= max(4., min(10., size * .4)):
            near.append((error, point, normal))
    if not near:
        return None
    _, point, normal = min(near, key=lambda row: row[0])
    if any(np.linalg.norm(other-point) > .25 for _, other, _ in near):
        return None
    return point, normal


@dataclass
class Track:
    point: np.ndarray
    normal: np.ndarray
    size: float
    stamp: float
    count: int = 1


class SquarePriority(SurfaceMemory):
    def __init__(self):
        super().__init__()
        self.tracks = []
        self.signals = []
        self.events = []
        self.cue_ids = set()
        self.stats = Counter()
        self.last_stamp = -math.inf
        self.decision_times = []
        self.looks = []

    def track_frame(self, detections, stamp, now, camera, *, confirmable=None):
        if stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        self.tracks = [tr for tr in self.tracks if stamp-tr.stamp <= .4]
        ready = []
        for index,(point, normal, size) in enumerate(detections):
            matches = [tr for tr in self.tracks if np.linalg.norm(tr.point-point) <= .10
                       and tr.normal@normal >= .82 and .6 <= size/tr.size <= 1.67]
            if not matches:
                self.tracks.append(Track(point, normal, size, stamp))
                continue
            tr = min(matches, key=lambda item: np.linalg.norm(item.point-point))
            if tr.stamp == stamp:
                continue
            tr.point = (tr.point+point)/2
            tr.normal = normal
            tr.stamp, tr.size = stamp, size
            tr.count += 1
            if confirmable is not None and index not in confirmable:
                continue
            self.signals.append((now, tr.point.copy(), normal.copy(), tr.count))
            ready.append(tr)
        if not ready or (self.looks and now-self.looks[-1]['t'] < 12):
            return
        def offset(tr):
            dx,dy = tr.point-camera[:2]
            angle = math.atan2(dy,dx)-camera[2]
            return math.atan2(math.sin(angle),math.cos(angle))
        for tr in sorted(ready, key=lambda item: (-item.count, abs(offset(item)))):
            if any(now-look['t'] < 90 and np.linalg.norm(look['point']-tr.point)<.8
                   and look['normal']@tr.normal >= .82 for look in reversed(self.looks)):
                continue
            angle = offset(tr)
            self.looks.append(dict(t=now, point=tr.point.copy(), normal=tr.normal.copy(),
                                   action='hold' if abs(angle)<=math.radians(8) else 'turn',
                                   yaw_delta_deg=round(math.degrees(angle),2),
                                   dwell_s=.5, frames=tr.count))
            break

    def pending(self, now):
        weak = super().pending(now)
        self.cue_ids = set()
        for t, point, normal, _ in reversed(self.signals):
            if now-t > 120:
                break
            idx = self.nearest(point, normal)
            if idx is not None:
                self.cue_ids.add(idx)
        return sorted(self.cue_ids.union(weak))

    def choose(self, snapshot, pose, now, focal, mapped_ids):
        self.decision_times.append(now)
        priority = [i for i in mapped_ids if i in self.cue_ids]
        if priority:
            self.stats['decisions_with_mapped_signal'] += 1
            choice = super().choose(snapshot, pose, now, focal, priority, report_covered=True)
            if choice:
                choice['source'] = 'square'
                self.events.append((now, choice.copy()))
                return choice
        weak = set(super().pending(now))
        choice = super().choose(snapshot, pose, now, focal, [i for i in mapped_ids if i in weak])
        if choice:
            choice['source'] = 'geometry'
            self.events.append((now, choice.copy()))
        return choice
