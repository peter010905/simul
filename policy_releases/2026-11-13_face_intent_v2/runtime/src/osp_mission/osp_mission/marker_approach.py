"""Approach decoded landmarks using only camera evidence and the observed map."""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

from osp_mission.edge_policy import visible
from osp_mission.policy import wrap


class MarkerApproach:
    # Aim inside the user's 1.5 m limit to allow for localization error.
    radius = 1.3
    witness_window_s = 1.0

    def __init__(self):
        self.hints = {}
        self.near = {}
        self.last_attempt = {}
        self.positions = {}
        self.range_witnesses = {}
        self.witness_runs = {}
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
        # A physical close-range witness uses odom-relative sensor geometry,
        # not cached map coordinates that may move during SLAM correction.
        readable=(confirmed and not evidence.get('provisional',False)
                  and float(evidence.get('min_edge_pixels',0))>=18.)
        odom_point=np.asarray((evidence.get('odom_point') or [])[:2],dtype=float)
        base=np.asarray(odom_robot,dtype=float) if odom_robot is not None else np.zeros(0)
        depth=np.asarray((evidence.get('camera_point') or []),dtype=float)
        laser=np.asarray((evidence.get('lidar_camera_point') or []),dtype=float)
        valid=(readable and odom_point.shape==(2,) and base.shape==(2,)
               and depth.shape==(3,) and laser.shape==(3,)
               and np.isfinite([*odom_point,*base,*depth,*laser]).all()
               and np.linalg.norm(depth-laser)<=.10)
        near_distance=float(np.linalg.norm(odom_point-base)) if valid else math.inf
        rows=self.witness_runs.setdefault(ident,[])
        rows[:]=[row for row in rows if 0.<=stamp-row['stamp']<=self.witness_window_s+1e-9]
        if valid and near_distance<=self.radius:
            if rows and np.linalg.norm(odom_point-np.asarray(rows[-1]['point_odom']))>.25:
                rows.clear()
            if not rows or stamp-rows[-1]['stamp']>=.15-1e-9:
                rows.append(dict(stamp=stamp,distance_m=near_distance,
                                 robot_odom=base.tolist(),point_odom=odom_point.tolist()))
                del rows[:-3]
            if len(rows)==3:
                self.range_witnesses[ident]=dict(source='three_confirmed_readable_near_odom_observations',
                    conservative_radius_m=self.radius,time_window_sim_s=self.witness_window_s,
                    observations=[dict(r) for r in rows])
        elif valid or not confirmed:
            # An explicit far measurement or unconfirmed ID breaks the run.
            self.witness_runs.pop(ident,None)
        # Missing/conflicting auxiliary measurements add no evidence. Earlier
        # individually validated frames expire after1s instead of being erased.


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
            for mapping in (self.positions,self.range_witnesses,self.witness_runs,self.hints,self.near,self.last_attempt):
                mapping.pop(ident,None)
        self.confirmed_ids=current

    def complete(self, targets):
        return len(targets) == 3 and all(self.verified(i, point) for i, point in targets.items())

    def report(self):
        return dict(required_actual_radius_m=1.5, operating_radius_m=self.radius,
                    near_confirmations=self.near,
                    approach_evidence=self.positions, completion_radius_m=1.5,
                    verification_mode='confirmed_id_and_three_readable_sensor_relative_frames',
                    sensor_proof_radius_m=self.radius, required_sensor_frames=3,
                    witness_window_sim_s=self.witness_window_s,
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
            for offset in (-.45, -.22, 0., .22, .45):
                if (robot-target)@direction <= 0.:
                    continue
                angle = math.atan2(direction[1], direction[0])+offset
                # .90 m + Nav2 .25 m stopping tolerance leaves room for the
                # 1.30 m sensor proof, inside the actual 1.50 m task limit.
                for standoff in (.75, .85, .9):
                    position = target+standoff*np.array([math.cos(angle), math.sin(angle)])
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
        self.last_attempt[ident] = now
        return result
