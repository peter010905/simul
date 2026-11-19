"""Bounded, map-only viewpoints for nearby compact objects in Mixed."""
import math
from collections import deque

import numpy as np
from scipy import ndimage

from osp_mission import _r32_navigation_core as r32


def local_boxes(grid, pose):
    """Fit the same four faces as R32, only around the current robot pose."""
    r, c = grid.index(*pose[:2])
    span = math.ceil(3.8 / grid.resolution)
    top, left = max(0, r-span), max(0, c-span)
    cells = grid.cells[top:r+span+1, left:c+span+1]
    if min(cells.shape, default=0) < 3:
        return []
    origin = grid.world(top-.5, left-.5)
    local = r32.Grid(cells, grid.resolution, (*origin, grid.origin[2]))
    groups = []
    r32.surface_samples(local, groups)
    boxes = []
    for faces in groups:
        points = np.array([p for p, _ in faces])
        center = points.mean(axis=0)
        widths = np.linalg.norm(points[[0, 2]]-points[[1, 3]], axis=1)
        if .32 <= max(widths) <= 1.1 and np.linalg.norm(center-pose[:2]) <= 2.7:
            boxes.append((center, faces))
    return boxes


def odom_point(point, transform):
    x, y, a = transform
    c, s = math.cos(a), math.sin(a)
    return np.array([x+c*point[0]-s*point[1], y+s*point[0]+c*point[1]])


class BoxViews:
    def __init__(self):
        self.attempts = deque(maxlen=36)
        self.last_choice = -math.inf

    def choose(self, grid, costmap, pose, transform, now, targets, veto):
        if transform is None or now-self.last_choice < 10:
            return None
        boxes = local_boxes(grid, pose)
        if not boxes:
            return None
        robot = np.array(pose[:2])
        free = (grid.cells >= 0) & (grid.cells <= 25)
        clearance = ndimage.distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1] * grid.resolution
        labels, _ = ndimage.label(clearance >= .32)
        start = grid.index(*robot)
        if not grid.inside(*start) or not labels[start]:
            return None
        nav = r32.connected_nav_cells(costmap, pose)
        best_score, best = -math.inf, None
        for center, faces in boxes:
            if any(np.linalg.norm(np.asarray(t[:2])-center)<.55 for t in targets.values()):
                continue
            center_odom = odom_point(center, transform)
            recent = [(angle,t) for old,angle,t in self.attempts
                      if now-t<120 and np.linalg.norm(old-center_odom)<.5]
            if len(recent)>=3:
                continue
            for i,(face, normal) in enumerate(faces):
                angle = math.atan2(normal[1],normal[0]) + transform[2]
                if any(math.cos(angle-a)>.70 for a,_ in recent):
                    continue
                # When possible move around the box to a corner, so a previously
                # hidden side enters the camera as well as the selected face.
                for j in ((2,3) if i<2 else (0,1)):
                    adjacent, side = faces[j]
                    corner = face + side*np.dot(adjacent-face,side)
                    for offset in (.64,.84):
                        view = corner + offset*(normal+side)
                        travel = float(np.linalg.norm(view-robot))
                        if travel>2.6:
                            continue
                        cell, nav_cell = grid.index(*view), costmap.index(*view)
                        if not grid.inside(*cell) or labels[cell]!=labels[start]:
                            continue
                        if not costmap.inside(*nav_cell) or not nav[nav_cell]:
                            continue
                        if not grid.line_clear(view,face+.12*normal):
                            continue
                        bearing = math.atan2(face[1]-view[1],face[0]-view[0])
                        # There is little to gain by sending a robot already
                        # looking at this face back to its current viewpoint.
                        if travel<.28 and math.cos(bearing-pose[2])>.88:
                            continue
                        behind = float(np.dot(center-robot, normal))/max(.1,np.linalg.norm(center-robot))
                        score = .8*behind - .65*travel - .25*np.linalg.norm(center-robot) - .8*len(recent)
                        goal = dict(x=float(view[0]),y=float(view[1]),yaw=bearing,distance=travel,
                                    stationary=False,scan_angle=0.,features=np.zeros(5),
                                    r32_kind='box_face_view',box_point=face.tolist(),
                                    face_center_odom=center_odom.tolist(),face_angle_odom=angle)
                        if score>best_score and not veto(goal):
                            best_score, best = score, goal
        return best

    def dispatched(self, goal, now):
        if 'face_center_odom' in goal:
            self.attempts.append((np.array(goal['face_center_odom']),goal['face_angle_odom'],now))
            self.last_choice=now
