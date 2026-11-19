"""Inspect all reachable faces of a confirmed, compact obstacle."""
import math

import numpy as np
from scipy import ndimage

from osp_mission.box_views import local_boxes, odom_point
from osp_mission.two_views import TwoViews, similar
from osp_mission import _r32_navigation_core as r32


class FullBox(TwoViews):
    def choose(self, grid, costmap, pose, transform, now, targets, veto):
        if transform is None or now - self.last_choice < 3.0:
            return None
        robot = np.asarray(pose[:2])
        nav = r32.connected_nav_cells(costmap, pose)
        free = (grid.cells >= 0) & (grid.cells <= 25)
        clearance = ndimage.distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1] * grid.resolution
        safe = clearance >= .32
        start = grid.index(*robot)
        if not grid.inside(*start) or not safe[start]:
            return None
        reachable, _ = ndimage.label(safe)
        best = None
        best_score = -math.inf
        for center, faces in local_boxes(grid, pose):
            center_odom = odom_point(center, transform)
            track = self.nearest(center_odom)
            if track is None or not track.confirmed:
                continue
            if any(np.linalg.norm(np.asarray(t[:2]) - center) < .55 for t in targets.values()):
                continue
            views = []
            for face, normal in faces:
                tangent = np.array([-normal[1], normal[0]])
                candidates = []
                for stand in (.9, 1.2, 1.5):
                    for shift in (0., -.25, .25):
                        point = face + stand * normal + shift * tangent
                        cell = grid.index(*point)
                        nav_cell = costmap.index(*point)
                        if not grid.inside(*cell) or not costmap.inside(*nav_cell):
                            continue
                        if reachable[cell] != reachable[start] or not nav[nav_cell]:
                            continue
                        if not grid.line_clear(point, face + .12 * normal):
                            continue
                        candidates.append((point, nav_cell))
                if not candidates:
                    break
                views.append(min(candidates, key=lambda item: np.linalg.norm(item[0] - robot)))
            if len(views) != 4:
                continue
            # Each face must connect around this object through observed, passable
            # costmap cells. Connection elsewhere in the room is insufficient.
            rows, cols = np.indices(nav.shape)
            coordinates = np.stack((rows, cols), axis=-1)
            center_cell = np.asarray(costmap.index(*center))
            around = np.linalg.norm(coordinates - center_cell, axis=-1) * costmap.resolution <= 2.0
            ring, _ = ndimage.label(nav & around)
            groups = [ring[cell] for _, cell in views]
            if not groups[0] or len(set(groups)) != 1:
                continue
            for (face, normal), (point, _) in zip(faces, views):
                angle = math.atan2(normal[1], normal[0]) + transform[2]
                if any(similar(angle, seen, 35) for seen in track.seen):
                    continue
                if any(now - stamp < 35 and np.linalg.norm(center_odom - old) < .4
                       and similar(angle, attempted, 35)
                       for old, attempted, stamp in self.retries):
                    continue
                bearing = math.atan2(face[1] - point[1], face[0] - point[0])
                goal = dict(x=float(point[0]), y=float(point[1]), yaw=bearing,
                            distance=float(np.linalg.norm(point - robot)), stationary=False,
                            scan_angle=0., features=np.zeros(5), r32_kind='box_face_view',
                            box_point=face.tolist(), face_center_odom=center_odom.tolist(),
                            face_angle_odom=angle)
                if veto(goal):
                    continue
                score = -goal['distance'] - .1 * abs(math.remainder(bearing - pose[2], 2 * math.pi))
                if score > best_score:
                    best_score, best = score, goal
        return best
