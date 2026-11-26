"""Retry a box face at a different safe view, then continue with a fresh face."""
import math

import numpy as np

from osp_mission.adaptive_faces import inspection_pose
from osp_mission.box_views import odom_point
from osp_mission.joint_surface import connecting_distances
from osp_mission.r32_drive import in_map


class BoxFaceRecovery:
    def __init__(self):
        self.active = None
        self.pending = None
        self.used = []

    def dispatched(self, goal):
        self.active = goal if goal.get('batch_box') is not None and not goal.get('stationary') else None
        # Normal planning also ends a pending recovery when no option fits.
        self.pending = None

    def navigation_result(self, status, reason, elapsed):
        goal, self.active = self.active, None
        if goal is None or status == 4 or (status != 6 and reason not in ('navigation_stalled', 'action_timeout')):
            return
        self.pending = (goal, elapsed)

    def choose(self, grid, costmap, safe, nav, distances, clearance, pose, transform,
               veto, ranked, elapsed, duration, excluded_surface):
        if self.pending is None:
            return None
        failed, failed_at = self.pending
        box = failed['batch_box']
        if elapsed-failed_at > 15. or duration-elapsed < 20.:
            return None
        self.used = [(b, t) for b, t in self.used if elapsed-t < 75.]
        if sum(math.dist(b, box) < .40 for b, _ in self.used) >= 3:
            return None
        start = grid.index(*pose[:2])
        nav_start = costmap.index(*pose[:2])
        if (not grid.inside(*start) or not safe[start] or not costmap.inside(*nav_start)
                or not nav[nav_start]):
            return None
        failed_view = failed.get('recovery_view_odom')
        failed_point = (in_map(('odom', *failed_view), transform) if failed_view is not None
                        else (failed['x'], failed['y']))

        def different_endpoint(goal):
            return math.dist((goal['x'], goal['y']), failed_point) >= .50 and not veto(goal)

        def routes(goals):
            cells = [grid.index(g['x'], g['y']) for g in goals]
            walks = connecting_distances(grid, costmap, safe, nav, start, cells, 3.5)
            return [(walks[cell], g) for cell, g in zip(cells, goals) if cell in walks]

        # This exceptional retry uses the original face geometry and current
        # clearance, visibility and reachability; it does not reset history.
        geometry = failed.get('recovery_geometry')
        if geometry is not None and not failed.get('box_recovery_same_face'):
            (x, y), yaw, width, offset = geometry
            center = np.asarray(in_map(('odom', x, y), transform))
            direction = yaw-transform[2]
            normal = np.array([math.cos(direction), math.sin(direction)])
            r, c = grid.index(*center)
            radius = math.ceil(.17/grid.resolution)
            region = grid.cells[max(0,r-radius):r+radius+1,max(0,c-radius):c+radius+1]
            if grid.inside(r,c) and (region >= 65).any() and not excluded_surface(center):
                fields = {k: failed[k] for k in ('r32_kind', 'surface_odom', 'normal_odom',
                          'scan_angle', 'face_inspection', 'recovery_geometry', 'batch_box',
                          'completed_box', 'previous_attempts') if k in failed}
                if 'fitted_face' in failed:
                    fields['fitted_face'] = failed['fitted_face']
                fields.update(surface_map=center.tolist(), stationary=False, features=np.zeros(5), local_batch=True)
                rejected = set()
                # Try the next legal viewpoint if SLAM and Nav2 masks don't
                # connect jointly to the first point.
                for _ in range(4):
                    view = inspection_pose(grid, costmap, nav, distances, clearance, center, normal,
                                           width, pose, fields,
                                           lambda g: not different_endpoint(g) or
                                           grid.index(g['x'],g['y']) in rejected,
                                           visibility_offset=offset)
                    if view is None:
                        break
                    goal = view[1]
                    connected = routes([goal])
                    if connected:
                        return self.pick(goal, connected[0][0], box, transform, True)
                    rejected.add(grid.index(goal['x'],goal['y']))

        alternatives = [g for _, g in ranked if g.get('batch_box') is not None
                        and math.dist(g['batch_box'], box) < .40 and not g.get('previous_attempts')
                        and (math.cos(g['normal_odom']-failed['normal_odom']) < .70)
                        and different_endpoint(g)]
        connected = routes(alternatives)
        if connected:
            walk, goal = min(connected, key=lambda item: ('fitted_face' not in item[1], item[0]))
            return self.pick(goal, walk, box, transform, False)
        return None

    @staticmethod
    def pick(goal, walk, box, transform, same_face):
        return dict(goal, local_batch=True, box_recovery_same_face=same_face,
                    box_recovery_walk_m=walk,
                    recovery_view_odom=odom_point((goal['x'], goal['y']), transform).tolist())

    def record_pick(self, goal, elapsed):
        if 'box_recovery_walk_m' in goal:
            self.used.append((goal['batch_box'], elapsed))
