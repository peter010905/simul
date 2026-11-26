"""Approach decoded landmarks using only camera evidence and the observed map."""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

from osp_mission.edge_policy import visible
from osp_mission.policy import wrap
from osp_mission.joint_surface import connecting_distances


class MarkerApproach:
    radius = 1.5

    def __init__(self, expected_marker_count=3):
        if type(expected_marker_count) is not int or not 0 <= expected_marker_count <= 1024:
            raise ValueError('expected_marker_count must be 0 (undisclosed) or a valid roster size')
        self.expected_marker_count = expected_marker_count
        self.hints = {}
        self.near = {}
        self.last_attempt = {}
        self.routed_views = {}
        self.positions = {}
        self.range_witnesses = {}
        self.confirmed_ids = set()

    def observe_pose(self, targets, robot, stamp, now):
        """Record fresh map-pose proximity for diagnostics, never completion."""
        robot = np.asarray(robot, dtype=float)
        if robot.shape != (2,) or not np.isfinite([*robot, stamp, now]).all() or not -.1 <= now-stamp <= .75:
            return
        for ident, point in targets.items():
            point = np.asarray(point, dtype=float)
            if point.shape != (2,) or not np.isfinite(point).all():
                continue
            distance = float(np.linalg.norm(robot-point))
            old = self.positions.get(ident)
            old_distance = float(np.linalg.norm(np.asarray(old['robot'])-point)) if old else math.inf
            if distance <= 1.5 and distance < old_distance:
                self.positions[ident] = dict(stamp=float(stamp), robot=robot.tolist(),
                    point=point.tolist(), distance_m=distance, source='map_to_base_tf_after_id_confirmation')

    def observe(self, evidence, robot, now, frame='map', confirmed=False, odom_robot=None):
        stamp = float(evidence['stamp'])
        point = np.asarray(evidence['map_point'][:2], dtype=float)
        robot = np.asarray(robot, dtype=float)
        if evidence.get('observation_frame') != frame or point.shape != (2,) or robot.shape != (2,):
            return
        if not np.isfinite([stamp, *point, *robot]).all() or not -.1 <= now-stamp <= .75:
            return
        ident = int(evidence['id'])
        old = self.hints.get(ident)
        if old and stamp <= old['stamp']:
            return
        self.hints[ident] = dict(point=point, observer=robot, stamp=stamp)
        # The detector publishes the decoded ID and measured base-relative
        # position together. Target-list publication can arrive one tick later.
        xy = evidence.get('close_base_xy')
        source = evidence.get('range_source')
        if (source in ('depth', 'lidar') and not evidence.get('provisional')
                and isinstance(xy, (list, tuple)) and len(xy) == 2
                and np.isfinite(xy).all() and ident not in self.range_witnesses):
            distance = math.hypot(*xy)
            if distance <= self.radius:
                self.range_witnesses[ident] = dict(
                    source='one_decoded_direct_range_frame', observations=[dict(
                        stamp=stamp, distance_m=distance, range_source=source, base_xy=list(xy))])

        distance = float(np.linalg.norm(point-robot))
        if distance <= self.radius:
            rows = self.near.setdefault(ident, [])
            if not rows or stamp-rows[-1]['stamp'] >= .15-1e-9:
                rows.append(dict(stamp=stamp, distance_m=distance, robot=robot.tolist(), point=point.tolist()))
                del rows[:-3]

    def verified(self, ident, target=None):
        # Map-pose proximity remains diagnostic: only sensor-relative frames
        # can grant completion. Keep that proof through later SLAM corrections.
        return ident in self.confirmed_ids and ident in self.range_witnesses

    def reconcile_targets(self, targets):
        current=set(targets)
        for ident in self.confirmed_ids-current:
            for mapping in (self.positions,self.range_witnesses,self.hints,self.near,self.last_attempt,self.routed_views):
                mapping.pop(ident,None)
        self.confirmed_ids=current

    def complete(self, targets):
        return (self.expected_marker_count > 0 and len(targets) == self.expected_marker_count
                and all(self.verified(i, point) for i, point in targets.items()))

    def report(self):
        return dict(expected_marker_count=self.expected_marker_count, required_actual_radius_m=1.5, operating_radius_m=self.radius,
                    near_confirmations=self.near,
                    approach_evidence=self.positions, completion_radius_m=1.5,
                    verification_mode='one_decoded_direct_range_frame',
                    sensor_proof_radius_m=self.radius, required_sensor_frames=1,
                    range_witness_evidence=self.range_witnesses,
                    verified_ids=sorted(i for i in self.positions.keys()|self.range_witnesses.keys() if self.verified(i)))

    def choose(self, data, resolution, origin, robot, heading, candidates, targets, now):
        robot = np.asarray(robot, dtype=float)
        origin = np.asarray(origin)
        safe = np.where((data == 0) & (distance_transform_edt(data <= 50)*resolution >= .32), 0, 100)
        options = []
        for ident, target in targets.items():
            if self.verified(ident, target) or now-self.last_attempt.get(ident, -math.inf) < 15.:
                continue
            hint = self.hints.get(ident)
            if hint is None:
                continue
            target = np.asarray(target)
            direction = hint['observer']-target
            length = np.linalg.norm(direction)
            if length < .3:
                continue
            choices = [dict(x=float(robot[0]), y=float(robot[1]), distance=0., stationary=True, features=np.zeros(5)), *candidates]
            # Additional viewpoints stay on the actually observed marker side.
            if (robot-target)@direction > 0.:
                for _, position in self.near_views(target, direction):
                    a, b = (robot-origin)/resolution, (position-origin)/resolution
                    if not visible(safe, a[1], a[0], np.array([b[1]]), np.array([b[0]]))[0]:
                        continue
                    distance = float(np.linalg.norm(position-robot))
                    choices.append(dict(x=float(position[0]), y=float(position[1]), distance=distance, features=np.array([0.,0.,distance,0.,0.])))
            for c in choices:
                position = np.array([c['x'], c['y']]); delta = position-target
                distance = np.linalg.norm(delta)
                if not .65 <= distance <= (self.radius if c.get('stationary') else .9):
                    continue
                if delta@direction/(distance*length) < .80:
                    continue
                end = target+.25*delta/distance
                a, b = (position-origin)/resolution, (end-origin)/resolution
                if not visible(data, a[1], a[0], np.array([b[1]]), np.array([b[0]]))[0]:
                    continue
                yaw = math.atan2(-delta[1], -delta[0])
                turn = abs(wrap(yaw-heading))
                selected = dict(c, yaw=yaw, approach_id=ident,
                                scan_angle=wrap(yaw-heading) if c.get('stationary') else 0.)
                options.append((c['distance']/.3+turn/.85, ident, selected))
        if not options:
            return None
        _, ident, result = min(options, key=lambda v:v[0])
        return result

    @staticmethod
    def near_views(target, direction):
        # Keep the existing approach geometry on the observed printed side.
        base_yaw = math.atan2(direction[1], direction[0])
        for offset in (-.45, -.22, 0., .22, .45):
            yaw = base_yaw + offset
            normal = np.array((math.cos(yaw), math.sin(yaw)))
            for standoff in (.75, .85, .9):
                yield offset, target + standoff * normal

    def routed_approach(self, grid, cost, safe, nav, distances, pose, targets, now, remaining, veto):
        """Use a measured viewing side and a connected route for the final outstanding proofs."""
        if not self.expected_marker_count or len(targets) != self.expected_marker_count:
            return None
        start = grid.index(*pose[:2])
        nav_start = cost.index(*pose[:2])
        if (not grid.inside(*start) or not safe[start] or not cost.inside(*nav_start) or not nav[nav_start]):
            return None
        # Same 65s Nav2 action timeout and 8s inspection reserve as the normal scheduler.
        limit = .20 * (min(remaining, 65.) - 8.)
        if limit <= 0:
            return None
        candidates = []
        for ident, point in targets.items():
            attempts = self.routed_views.get(ident, ())
            hint = self.hints.get(ident)
            if (ident not in self.confirmed_ids or self.verified(ident) or hint is None
                    or now - self.last_attempt.get(ident, -math.inf) < 15. or len(attempts) >= 2):
                continue
            target = np.asarray(point, dtype=float)
            if not np.isfinite(target).all() or np.linalg.norm(target-hint['point']) > .25:
                continue
            direction = hint['observer'] - target
            if not np.isfinite(direction).all() or np.linalg.norm(direction) < .3:
                continue
            for offset, position in self.near_views(target, direction):
                # Do not send the same side twice if an approach failed to produce evidence.
                if offset in attempts:
                    continue
                cell = grid.index(*position)
                nc = cost.index(*position)
                if (not grid.inside(*cell) or not safe[cell] or not 0 < distances[cell] <= limit
                        or not cost.inside(*nc) or not nav[nc]):
                    continue
                delta = position - target
                end = grid.index(*(target + .25 * delta / np.linalg.norm(delta)))
                if not visible(grid.cells, *cell, np.array([end[0]]), np.array([end[1]]))[0]:
                    continue
                yaw = math.atan2(-delta[1], -delta[0])
                goal = dict(x=float(position[0]), y=float(position[1]), yaw=yaw,
                            approach_id=ident, routed_offset=offset, scan_angle=0., stationary=False)
                if not veto(goal):
                    candidates.append((cell, goal))
        if not candidates:
            return None
        routes = connecting_distances(grid, cost, safe, nav, start, (cell for cell, _ in candidates), limit)
        scored = [(routes[cell]/.3 + abs(wrap(goal['yaw']-pose[2]))/.85, routes[cell], goal)
                  for cell, goal in candidates if cell in routes]
        if not scored:
            return None
        _, walk, goal = min(scored, key=lambda row: row[0])
        return dict(goal, distance=walk, routed_approach_m=walk, features=np.array([0., 0., walk, 0., 0.]))

    def dispatched(self, goal, now):
        ident = goal.get('approach_id')
        if ident is None:
            return
        self.last_attempt[ident] = now
        self.routed_dispatched(goal, now)

    def routed_dispatched(self, goal, now):
        if 'routed_offset' in goal:
            ident = goal['approach_id']
            self.routed_views.setdefault(ident, []).append(goal['routed_offset'])
            self.last_attempt[ident] = now
