"""Treat mapped right-angle returns as provisional rectangular objects."""
import math
from collections import deque
from dataclasses import dataclass, field

import cv2
import numpy as np
from scipy import ndimage

from osp_mission import _r32_navigation_core as r32
from osp_mission.box_views import odom_point
from osp_mission.thorough_box import ThoroughBox
from osp_mission.two_views import similar


def local_corners(grid, pose):
    """Pair measured, finite orthogonal LiDAR edges; infer no unseen wall pixels."""
    row, col = grid.index(*pose[:2])
    span = math.ceil(3.0 / grid.resolution)
    top, left = max(0, row-span), max(0, col-span)
    bottom, right = min(grid.cells.shape[0], row+span+1), min(grid.cells.shape[1], col+span+1)
    occupied = (grid.cells[top:bottom, left:right] >= 65).astype(np.uint8) * 255
    if min(occupied.shape, default=0) < 12 or np.count_nonzero(occupied) < 12:
        return []
    minimum = max(7, round(.42/grid.resolution))
    raw = cv2.HoughLinesP(occupied, 1, math.pi/180, threshold=minimum,
                          minLineLength=minimum, maxLineGap=max(1, round(.10/grid.resolution)))
    if raw is None:
        return []
    segments = []
    for x1, y1, x2, y2 in raw[:, 0]:
        a, b = grid.world(top+y1, left+x1), grid.world(top+y2, left+x2)
        length = float(np.linalg.norm(b-a))
        if .42 <= length <= 2.6:
            segments.append((a, b, length))
    segments.sort(key=lambda item: -item[2])
    found = []
    for i, (a, b, length_a) in enumerate(segments[:32]):
        for c, d, length_b in segments[i+1:32]:
            direction_a, direction_b = (b-a)/length_a, (d-c)/length_b
            if abs(direction_a @ direction_b) > .28:
                continue
            ends = [(float(np.linalg.norm(p-q)), p, q, other_a, other_b)
                    for p, other_a in ((a, b), (b, a))
                    for q, other_b in ((c, d), (d, c))]
            gap, p, q, far_a, far_b = min(ends, key=lambda item: item[0])
            if gap > .22:
                continue
            corner = (p+q)/2
            if np.linalg.norm(corner-np.asarray(pose[:2])) > 2.8:
                continue
            arm_a, arm_b = (far_a-corner)/np.linalg.norm(far_a-corner), (far_b-corner)/np.linalg.norm(far_b-corner)
            # A corner is one hypothesis, even when Hough draws several nearby lines.
            if any(np.linalg.norm(corner-old[0]) < .30 for old in found):
                continue
            found.append((corner, ((arm_a, length_a), (arm_b, length_b))))
    return found[:12]


@dataclass
class CornerTrack:
    center: np.ndarray
    last: float
    hits: int = 1
    seen: list = field(default_factory=list)
    exposures: dict = field(default_factory=dict)


class AngularBox(ThoroughBox):
    def __init__(self):
        super().__init__()
        self.corner_tracks = []
        self.corner_attempts = deque(maxlen=48)

    def corner_track(self, center):
        track = min(self.corner_tracks, key=lambda t: np.linalg.norm(t.center-center), default=None)
        return track if track is not None and np.linalg.norm(track.center-center) < .40 else None

    def observe(self, grid, pose, endpoints, transform, stamp, fov):
        previous = self.last_frame
        super().observe(grid, pose, endpoints, transform, stamp, fov)
        if self.last_frame == previous or transform is None:
            return
        self.corner_tracks = [t for t in self.corner_tracks if stamp-t.last < 90.]
        camera = np.asarray(pose[:2])
        for corner, arms in local_corners(grid, pose):
            mapped = odom_point(corner, transform)
            track = self.corner_track(mapped)
            if track is None:
                track = CornerTrack(mapped, stamp)
                self.corner_tracks.append(track)
            elif stamp-track.last >= .35:
                track.hits = min(12, track.hits+1) if stamp-track.last <= 8 else 1
            track.last = stamp
            for arm, length in arms:
                face = corner + arm*min(.45, length*.55)
                for normal in (np.array([-arm[1], arm[0]]), np.array([arm[1], -arm[0]])):
                    delta = camera-face
                    distance = float(np.linalg.norm(delta))
                    if not .7 <= distance <= 1.5 or delta @ normal < .5*distance:
                        continue
                    bearing = math.atan2(-delta[1], -delta[0])
                    if not similar(bearing, pose[2], min(27, math.degrees(fov)/2-3)):
                        continue
                    along = (endpoints-face) @ arm
                    plane = np.abs((endpoints-face) @ normal) < .09
                    support = plane & (along >= -.25) & (along <= min(.6, length))
                    if support.sum() < 4 or np.ptp(along[support]) < .2:
                        continue
                    angle = math.atan2(normal[1], normal[0]) + transform[2]
                    key = next((a for a in track.exposures if similar(angle, a, 25)), angle)
                    first, last = track.exposures.get(key, (stamp, stamp))
                    if stamp-last > 1.7:
                        first = stamp
                    track.exposures[key] = first, stamp
                    if stamp-first >= .4 and not any(similar(angle, a, 30) for a in track.seen):
                        track.seen.append(angle)

    def choose(self, grid, costmap, pose, transform, now, targets, veto):
        proven = super().choose(grid, costmap, pose, transform, now, targets, veto)
        if proven is not None:
            return proven
        if transform is None or now-self.last_choice < 3.:
            return None
        robot = np.asarray(pose[:2])
        connected, distances = r32.nav_goal_reachability(
            grid, costmap, pose, grid.reachable(pose[:2], .32))
        best = None
        glance = None
        for corner, arms in local_corners(grid, pose):
            center_odom = odom_point(corner, transform)
            track = self.corner_track(center_odom)
            if track is None or track.hits < 2 or now-track.last > 30:
                continue
            if any(np.linalg.norm(np.asarray(t[:2])-corner) < .65 for t in targets.values()):
                continue
            for arm, length in arms:
                face = corner + arm*min(.45, length*.55)
                for normal in (np.array([-arm[1], arm[0]]), np.array([arm[1], -arm[0]])):
                    angle = math.atan2(normal[1], normal[0]) + transform[2]
                    if any(similar(angle, old, 30) for old in track.seen):
                        continue
                    if any(now-stamp < 35 and np.linalg.norm(center_odom-old) < .4 and similar(angle, attempted, 30)
                           for old, attempted, stamp in self.corner_attempts):
                        continue
                    if sum(now-stamp < 120 and np.linalg.norm(center_odom-old) < .4
                           and similar(angle, attempted, 30)
                           for old, attempted, stamp in self.corner_attempts) >= 2:
                        continue
                    delta = face-robot
                    face_range = float(np.linalg.norm(delta))
                    if .7 <= face_range <= 1.8 and (robot-face) @ normal > .45*face_range:
                        bearing_now = math.atan2(delta[1], delta[0])
                        turn = abs(math.remainder(bearing_now-pose[2], 2*math.pi))
                        start_cell = grid.index(*robot)
                        # Turning in place can clarify a partly mapped surface;
                        # no free-space claim is made beyond the robot footprint.
                        if .30 <= turn <= 1.5 and grid.inside(*start_cell) and connected[start_cell]:
                            look = dict(x=float(robot[0]), y=float(robot[1]), yaw=bearing_now,
                                        distance=0., stationary=True, scan_angle=0., features=np.zeros(5),
                                        r32_kind='box_face_view', box_view_scope='corner_glance',
                                        box_point=face.tolist(), face_center_odom=center_odom.tolist(),
                                        face_angle_odom=angle)
                            score = turn+.15*face_range
                            if not veto(look) and (glance is None or score < glance[0]):
                                glance = score, look
                    for stand in (1.0, 1.25, 1.45):
                        view = face + stand*normal
                        cell = grid.index(*view)
                        if not grid.inside(*cell) or not connected[cell] or not np.isfinite(distances[cell]):
                            continue
                        if not grid.line_clear(view, face+.12*normal):
                            continue
                        bearing = math.atan2(face[1]-view[1], face[0]-view[0])
                        travel = float(np.linalg.norm(view-robot))
                        goal = dict(x=float(view[0]), y=float(view[1]), yaw=bearing,
                                    distance=travel, stationary=False, scan_angle=0.,
                                    features=np.zeros(5), r32_kind='box_face_view',
                                    box_view_scope='corner_hypothesis', box_point=face.tolist(),
                                    face_center_odom=center_odom.tolist(), face_angle_odom=angle)
                        if veto(goal):
                            continue
                        score = distances[cell]*grid.resolution + .18*abs(math.remainder(bearing-pose[2], 2*math.pi))
                        if best is None or score < best[0]:
                            best = score, goal
        return best[1] if best else glance[1] if glance else None

    def dispatched(self, goal, now):
        super().dispatched(goal, now)
        if goal.get('box_view_scope') in ('corner_hypothesis', 'corner_glance'):
            self.corner_attempts.append((np.asarray(goal['face_center_odom']), goal['face_angle_odom'], now))
