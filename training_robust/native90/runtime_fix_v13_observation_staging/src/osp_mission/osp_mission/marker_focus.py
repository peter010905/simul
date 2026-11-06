"""Bounded follow-up views for real, accepted RGB detections awaiting confirmation."""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

from osp_mission.edge_policy import visible
from osp_mission.policy import wrap


class MarkerFocus:
    def __init__(self):
        self.hints = {}
        self.attempts = {}
        self.last_attempt = {}
        self.interruptions = {}

    def observe(self, evidence, observer, now, frame='map'):
        if evidence.get('observation_frame') != frame:
            return
        stamp = float(evidence['stamp'])
        point = np.asarray(evidence['map_point'][:2], dtype=float)
        observer = np.asarray(observer, dtype=float)
        if point.shape != (2,) or observer.shape != (2,):
            return
        if not np.isfinite([stamp, *point, *observer]).all() or not -.1 <= now-stamp <= .75:
            return
        ident = int(evidence['id'])
        old = self.hints.get(ident)
        if old is not None and stamp <= old['stamp']:
            return
        self.hints[ident] = dict(point=point, observer=observer, stamp=stamp,
                                count=1 if old is None else old['count']+1)
        return True

    def request_interrupt(self, ident, confirmed, now):
        """Pause an underway navigation once per fresh, unconfirmed image ID."""
        hint = self.hints.get(ident)
        if hint is None or ident in confirmed or ident in self.interruptions:
            return False
        if len(self.interruptions) >= 6 or self.attempts.get(ident, 0) >= 2:
            return False
        if hint['count'] >= 3 or not 0. <= now-hint['stamp'] <= .75:
            return False
        if not .7 <= np.linalg.norm(hint['observer']-hint['point']) <= 5.5:
            return False
        self.interruptions[ident] = now
        return True

    def choose(self, data, resolution, origin, robot, heading, candidates, confirmed, now):
        """Return a normal safe waypoint or a stationary look; never a target pose."""
        robot = np.asarray(robot, dtype=float)
        choices = []
        safe_grid = None
        for ident, hint in self.hints.items():
            age = now-hint['stamp']
            # Let the normal publication of three confirmations settle first.
            if ident in confirmed or hint['count'] >= 3 or not 1. <= age <= 600.:
                continue
            if self.attempts.get(ident, 0) >= 4 or now-self.last_attempt.get(ident, -math.inf) < 30.:
                continue
            target = hint['point']
            observed_direction = hint['observer']-target
            observed_range = np.linalg.norm(observed_direction)
            if observed_range < .5:
                continue
            stationary = dict(x=float(robot[0]), y=float(robot[1]), distance=0.,
                              features=np.zeros(5), stationary=True)
            targeted = []
            # Normal exploration keeps one point per metre-sized tile. A real
            # image can justify closer viewpoints that this sparse grid missed.
            if safe_grid is None:
                clearance = distance_transform_edt(data <= 50)*resolution
                safe_grid = np.where((data == 0) & (clearance >= .32), 0, 100)
            for standoff in (.9, 1.3, 1.8, 2.3):
                # A direct shortcut must remain on the observed side. Sparse
                # SLAM can omit the carrier cell; do not cut through the marker.
                if (robot-target) @ observed_direction <= 0.:
                    continue
                position = target+observed_direction/observed_range*standoff
                distance = float(np.linalg.norm(position-robot))
                if not .3 <= distance <= 4.:
                    continue
                start = (robot-np.asarray(origin))/resolution
                stop = (position-np.asarray(origin))/resolution
                cell = np.floor(stop).astype(int)
                if not (0 <= cell[1] < data.shape[0] and 0 <= cell[0] < data.shape[1]) or safe_grid[cell[1],cell[0]]:
                    continue
                if not visible(safe_grid,start[1],start[0],np.array([stop[1]]),np.array([stop[0]]))[0]:
                    continue
                targeted.append(dict(x=float(position[0]),y=float(position[1]),distance=distance,
                                     features=np.array([0.,0.,distance,0.,0.]),targeted=True))
            for candidate in [stationary, *candidates, *targeted]:
                if candidate['distance'] > 4.:
                    continue
                position = np.array([candidate['x'], candidate['y']])
                direction = position-target
                distance = np.linalg.norm(direction)
                limit = observed_range+.15 if candidate.get('stationary') else observed_range*.95+.05
                if not .7 <= distance <= min(3., limit):
                    continue
                # Stay on the side from which an actual readable image was taken.
                alignment = float(direction @ observed_direction / (distance*observed_range))
                if alignment < .85:
                    continue
                endpoint = target+direction/distance*.25
                start = (position-np.asarray(origin))/resolution
                stop = (endpoint-np.asarray(origin))/resolution
                if not visible(data, start[1], start[0], np.array([stop[1]]), np.array([stop[0]]))[0]:
                    continue
                yaw = math.atan2(-direction[1], -direction[0])
                turn = abs(wrap(yaw-heading))
                cost = candidate['distance']/.3 + turn/.85 + 2.*distance + 3.*(1.-alignment)
                if candidate.get('stationary') and self.attempts.get(ident,0)>0 and distance>1.5:
                    cost+=8.  # after a failed distant look, prefer a closer safe view
                features = candidate['features'].copy()
                features[3] = turn
                selected = dict(candidate, yaw=yaw, scan_angle=0., focus_id=ident, features=features)
                if candidate.get('stationary'):
                    selected['scan_angle'] = wrap(yaw-heading)
                choices.append((cost, ident, selected))
        if not choices:
            return None
        _, ident, selected = min(choices, key=lambda row: row[0])
        self.attempts[ident] = self.attempts.get(ident, 0)+1
        self.last_attempt[ident] = now
        return selected
