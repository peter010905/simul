"""Approach decoded landmarks using only camera evidence and the observed map."""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

from osp_mission.edge_policy import visible
from osp_mission.policy import wrap


class MarkerApproach:
    # Aim inside the user's 1.5 m limit to allow for localization error.
    radius = 1.3

    def __init__(self):
        self.hints = {}
        self.near = {}
        self.last_attempt = {}

    def observe(self, evidence, robot, now, frame='map'):
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
        distance = float(np.linalg.norm(point-robot))
        if distance <= self.radius:
            rows = self.near.setdefault(ident, [])
            if not rows or stamp-rows[-1]['stamp'] >= .15-1e-9:
                rows.append(dict(stamp=stamp, distance_m=distance, robot=robot.tolist(), point=point.tolist()))
                del rows[:-3]

    def verified(self, ident):
        return len(self.near.get(ident, ())) >= 3

    def complete(self, targets):
        return len(targets) == 3 and all(self.verified(i) for i in targets)

    def report(self):
        return dict(required_actual_radius_m=1.5, operating_radius_m=self.radius,
                    near_confirmations=self.near,
                    verified_ids=sorted(i for i in self.near if self.verified(i)))

    def choose(self, data, resolution, origin, robot, heading, candidates, targets, now):
        robot = np.asarray(robot, dtype=float)
        origin = np.asarray(origin)
        safe = np.where((data == 0) & (distance_transform_edt(data <= 50)*resolution >= .32), 0, 100)
        options = []
        for ident, target in targets.items():
            if self.verified(ident) or now-self.last_attempt.get(ident, -math.inf) < 15.:
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
                for standoff in (.9, 1.05, 1.2):
                    position = target+standoff*np.array([math.cos(angle), math.sin(angle)])
                    a, b = (robot-origin)/resolution, (position-origin)/resolution
                    if not visible(safe, a[1], a[0], np.array([b[1]]), np.array([b[0]]))[0]:
                        continue
                    distance = float(np.linalg.norm(position-robot))
                    choices.append(dict(x=float(position[0]), y=float(position[1]), distance=distance, features=np.array([0.,0.,distance,0.,0.])))
            for c in choices:
                position = np.array([c['x'], c['y']]); delta = position-target
                distance = np.linalg.norm(delta)
                if not .65 <= distance <= (self.radius if c.get('stationary') else 1.2):
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
