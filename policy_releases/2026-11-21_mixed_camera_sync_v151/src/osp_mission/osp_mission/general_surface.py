"""Bounded follow-up for an unviewed surface near a completed brief look.

Object shape is not inferred. Only measured occupied/free boundaries supply
candidate faces; the global costmap and Nav2 retain motion safety authority.
"""
import math
from collections import deque

import numpy as np

from osp_mission import _r32_navigation_core as r32
from osp_mission.edge_policy import boundary_faces, visible


class GeneralSurfaceFollowup:
    def __init__(self):
        self.attempts = deque(maxlen=12)
        self.used_glances = set()
        self.last_choice = -math.inf
        self.maximum_goals = 3
        self.maximum_travel_m = 5.
        self.travel_committed_m = 0.

    def choose(self, grid, costmap, pose, viewed, glance, glance_key, now,
               remaining, veto):
        if (glance is None or glance_key in self.used_glances or
                len(self.attempts) >= self.maximum_goals or
                self.travel_committed_m >= self.maximum_travel_m or
                now-self.last_choice < 20. or remaining < 60. or
                viewed is None or viewed.shape != (4,)+grid.cells.shape):
            return None
        # The first look must have finished normally. An interrupted turn is
        # never evidence for spending another navigation goal.
        if (glance.get('reason') != 'processed_frame_dwell' or
                not 0. <= now-glance.get('started', -math.inf) <= 45.):
            return None
        camera = np.asarray(glance['map_camera'][:2], dtype=float)
        robot = np.asarray(pose[:2], dtype=float)
        if not np.isfinite([*camera, *robot]).all():
            return None
        faces = boundary_faces(grid.cells, grid.resolution)
        if not len(faces['side']):
            return None
        points = np.column_stack((faces['px'], faces['py']))*grid.resolution + np.asarray(grid.origin[:2])
        normals = np.column_stack((faces['nx'], faces['ny']))
        unseen = ~viewed[faces['side'], faces['y'], faces['x']]
        nearby = np.linalg.norm(points-camera, axis=1) <= 1.8
        ids = np.flatnonzero(unseen & nearby)
        if not len(ids):
            return None
        connected, distances = r32.nav_goal_reachability(
            grid, costmap, pose, grid.reachable(pose[:2], .32))
        best = None
        # Sample a bounded number of measured boundary cells. This permits
        # irregular objects without classifying walls as four-sided boxes.
        for index in ids[::max(1, len(ids)//160)]:
            face, normal = points[index], normals[index]
            if np.linalg.norm(normal) < .9:
                continue
            if any(np.linalg.norm(face-old) < .55 and now-stamp < 90.
                   for old, stamp in self.attempts):
                continue
            for stand in (1.0, 1.3):
                position = face+stand*normal
                travel = float(np.linalg.norm(position-robot))
                if travel > 2.5 or self.travel_committed_m+travel > self.maximum_travel_m:
                    continue
                cell = grid.index(*position)
                if not grid.inside(*cell) or not connected[cell] or not np.isfinite(distances[cell]):
                    continue
                if not grid.line_clear(position, face+.12*normal):
                    continue
                # Explicit camera-side line of sight through known free cells.
                start = grid.index(*position)
                end = grid.index(*(face+.12*normal))
                if not visible(grid.cells, start[0]+.5, start[1]+.5,
                               np.array([end[0]+.5]), np.array([end[1]+.5]))[0]:
                    continue
                yaw = math.atan2(face[1]-position[1], face[0]-position[0])
                goal = dict(x=float(position[0]), y=float(position[1]), yaw=yaw,
                            distance=travel, stationary=False,
                            scan_angle=0., features=np.zeros(5),
                            r32_kind='general_surface_view',
                            surface_point=face.tolist(), glance_key=glance_key)
                if veto(goal):
                    continue
                score = distances[cell]*grid.resolution + .4*stand
                if best is None or score < best[0]:
                    best = score, goal
        return best[1] if best else None

    def dispatched(self, goal, now):
        if goal.get('r32_kind') != 'general_surface_view':
            return
        self.used_glances.add(goal['glance_key'])
        self.attempts.append((np.asarray(goal['surface_point']), now))
        self.travel_committed_m += goal['distance']
        self.last_choice = now
