"""Associate occupancy-grid surface fragments with camera-tracked box faces in odom."""
from dataclasses import dataclass
import math

import numpy as np
from osp_mission.box_views import odom_point


@dataclass
class Face:
    anchor: np.ndarray
    normal: np.ndarray
    yaw: float
    half_width: float
    observed: bool


@dataclass(eq=False)
class Fit:
    center: np.ndarray
    faces: list
    updated: float


class TrackedFaces:
    def __init__(self):
        self.boxes = []

    def update(self, fitted, transform, now):
        # A short gap in the local rectangle fit need not discard fresh camera evidence.
        self.boxes = [box for box in self.boxes if 0 <= now-box.updated <= 45.]
        for center, faces, seen in fitted:
            mapped = odom_point(center, transform)
            nearest = min(self.boxes, key=lambda box: np.linalg.norm(box.center-mapped), default=None)
            if nearest is not None and np.linalg.norm(nearest.center-mapped) < .30:
                self.boxes.remove(nearest)
            records = []
            for face, normal in faces:
                angle = math.atan2(normal[1], normal[0]) + transform[2]
                tangent = np.array([-normal[1], normal[0]])
                half_width = max(abs((p-center) @ tangent) for p, _ in faces)
                observed = any(math.cos(angle-a) >= math.cos(math.radians(35)) for a in seen)
                records.append(Face(odom_point(face, transform), np.array([math.cos(angle), math.sin(angle)]),
                                    angle, half_width, observed))
            self.boxes.append(Fit(mapped, records, now))

    def match(self, point, grid_yaw, resolution):
        owner = None
        direction = np.array([math.cos(grid_yaw), math.sin(grid_yaw)])
        # Tolerate the occupied/free half-cell offset and ordinary rectangle-fit noise.
        tolerance = min(.13, max(.075, 2*resolution))
        for box in self.boxes:
            if np.linalg.norm(point-box.center) > 1.0:
                continue
            matches = []
            for face in box.faces:
                alignment = float(face.normal @ direction)
                if alignment < .45:
                    continue
                delta = point-face.anchor
                plane = abs(float(delta @ face.normal))
                along = abs(float(delta[1]*face.normal[0] - delta[0]*face.normal[1]))
                if plane <= tolerance and along <= face.half_width + resolution:
                    matches.append((plane + max(0., along-face.half_width), -alignment, face))
            if not matches:
                continue
            if owner is not None:
                return None  # Points shared by separate fitted objects retain the generic check.
            # At a corner, retain the opportunity to inspect a still-unseen adjacent face.
            owner = min(matches, key=lambda m: (m[2].observed, m[0], m[1]))[2]
        return owner

    def representative(self, grid, remaining, dy, dx, transform, grid_yaw):
        def locate(cell):
            r,c = cell
            point = grid.world(r+.5*dy, c+.5*dx)
            anchor = odom_point(point, transform)
            return point, anchor, self.match(anchor, grid_yaw, grid.resolution)

        mid = locate(remaining[len(remaining)//2])
        if mid[2] is None or not mid[2].observed:
            return mid
        # A bin may contain cells on an adjoining face or an unrelated object.
        available = []
        for cell in remaining:
            sample = locate(cell)
            if sample[2] is None or not sample[2].observed:
                available.append(sample)
        if len(available)*grid.resolution >= .15:
            return available[len(available)//2]
        return None
