"""Confirm compact objects from independent lidar views and remember camera coverage."""
import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from osp_mission.box_views import BoxViews, local_boxes, odom_point


def similar(a, b, degrees):
    return math.cos(a-b) > math.cos(math.radians(degrees))


@dataclass
class Track:
    center: np.ndarray
    last: float
    evidence: list = field(default_factory=list)
    exposures: dict = field(default_factory=dict)
    seen: list = field(default_factory=list)
    confirmed: bool = False


class TwoViews(BoxViews):
    def __init__(self):
        super().__init__()
        self.tracks = []
        self.retries = deque(maxlen=48)
        self.last_frame = -math.inf

    def nearest(self, point):
        track = min(self.tracks, key=lambda t: np.linalg.norm(t.center-point), default=None)
        if track is not None and np.linalg.norm(track.center-point) < .40:
            return track
        return None

    def observe(self, grid, pose, endpoints, transform, stamp, fov):
        # Exposure timestamps also prevent the same image being counted twice.
        if stamp-self.last_frame < .4:
            return
        self.last_frame = stamp
        self.corner_half_fov = max(0.,min(math.radians(25), fov/2-math.radians(4)))
        self.tracks = [t for t in self.tracks if stamp-t.last < 600.]
        for center, faces in local_boxes(grid, pose):
            mapped = odom_point(center, transform)
            observer = np.asarray(pose[:2])
            bearing = math.atan2(*(observer-center)[::-1]) + transform[2]
            track = self.nearest(mapped)
            for face, normal in faces:
                delta = observer-face
                distance = np.linalg.norm(delta)
                if distance < .10 or delta @ normal < .4*distance:
                    continue
                tangent = np.array([-normal[1], normal[0]])
                half_width = max(abs((p-center) @ tangent) for p,_ in faces)
                along = (endpoints-face) @ tangent
                on_plane = (abs((endpoints-face) @ normal) < .085) & (abs(along) <= half_width+.06)
                if on_plane.sum() < 4 or np.ptp(along[on_plane]) < .20:
                    continue
                # A mapped free patch beyond one end supports a finite edge.
                ends = [face+side*tangent*(half_width+.16)+.02*normal for side in (-1,1)]
                if not any(grid.inside(*grid.index(*p)) and 0 <= grid.cells[grid.index(*p)] <= 25 for p in ends):
                    continue
                if track is None:
                    track = Track(mapped, stamp)
                    self.tracks.append(track)
                track.last = stamp
                angle = math.atan2(normal[1],normal[0]) + transform[2]
                previous = track.evidence
                # Distinct viewpoints AND adjacent supported surface normals.
                if any(not similar(bearing,b,25) and abs(math.cos(angle-a)) < .5
                       and stamp-t >= .4 for a,b,t in previous):
                    track.confirmed = True
                if not any(similar(angle,a,20) and similar(bearing,b,12) for a,b,_ in previous):
                    previous.append((angle,bearing,stamp))
                direction = math.atan2(-delta[1],-delta[0])
                if distance <= 1.5 and similar(direction,pose[2],min(27,math.degrees(fov)/2-3)):
                    key = next((a for a in track.exposures if similar(angle,a,30)), angle)
                    first,last = track.exposures.get(key,(stamp,stamp))
                    if stamp-last > 1.7:
                        first = stamp
                    track.exposures[key] = first,stamp
                    if stamp-first >= .40 and not any(similar(angle,a,35) for a in track.seen):
                        track.seen.append(angle)

    def choose(self, grid, costmap, pose, transform, now, targets, veto):
        if not any(t.confirmed for t in self.tracks):
            return None

        def unavailable(goal):
            center = np.asarray(goal['face_center_odom'])
            track = self.nearest(center)
            angle = goal['face_angle_odom']
            if track is None or not track.confirmed or any(similar(angle,a,35) for a in track.seen):
                return True
            if any(now-t < 35 and np.linalg.norm(center-c)<.4 and similar(angle,a,35)
                   for c,a,t in self.retries):
                return True
            return veto(goal)

        return super().choose(grid, costmap, pose, transform, now, targets, unavailable)

    def dispatched(self, goal, now):
        if 'face_center_odom' in goal:
            self.retries.append((np.asarray(goal['face_center_odom']), goal['face_angle_odom'], now))
            self.last_choice = now
