"""Bounded looks at unobserved wall-like surfaces during exploration transit.

Only the live occupancy map, processed-camera coverage and TF are inputs.
Wall-like is the existing observed-floor classification, not a semantic truth.
"""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

from osp_mission import _r32_navigation_core as r32
from osp_mission.center_wall import surface_cells
from osp_mission.edge_policy import boundary_faces, visible
from osp_mission.near_pass_inspection import NearPassInspection
from osp_mission.policy import wrap


class WallTransitInspection(NearPassInspection):
    maximum_camera_range_m = 3.5
    maximum_turn_rad = math.radians(100)
    maximum_total_sim_s = 35.
    maximum_looks = 6

    def choose(self, data, resolution, origin, camera, robot, viewed, focal, fov,
               odom_camera, now, elapsed, remaining, protected=False):
        if (protected or self.active is not None or elapsed < 20 or remaining < 55
                or len(self.history) >= self.maximum_looks or now-self.last_started < 25
                or self.spent >= self.maximum_total_sim_s):
            return None
        if not np.isfinite([resolution, *origin, *camera, *robot, focal, fov,
                            *odom_camera, now, elapsed, remaining]).all():
            return None
        if resolution <= 0 or focal <= 0 or not 0 < fov < math.pi:
            return None
        if viewed.shape != (4,)+data.shape:
            raise ValueError('Four physical boundary-face states required')
        base = np.floor((np.asarray(robot)-origin)/resolution).astype(int)
        if not (0 <= base[1] < data.shape[0] and 0 <= base[0] < data.shape[1]):
            return None
        if data[base[1], base[0]] != 0:
            return None
        if distance_transform_edt(data == 0)[base[1], base[0]]*resolution < .32:
            return None
        # A private eligibility mask; never write to the coverage owner.
        walls = np.zeros_like(viewed, dtype=bool)
        grid = r32.Grid(data, resolution, (*origin, 0.))
        for (kind, _, side, _, _), cells in surface_cells(grid, robot).items():
            if kind == 'wall':
                for row, col in cells:
                    walls[side, row, col] = True
        f = boundary_faces(data, resolution)
        dy = (camera[1]-origin[1])/resolution-f['py']
        dx = (camera[0]-origin[0])/resolution-f['px']
        lengths = np.hypot(dy, dx)
        distances = lengths*resolution
        incidence = (dy*f['ny']+dx*f['nx'])/np.maximum(lengths, 1e-9)
        bearing = (np.arctan2(-dy, -dx)-camera[2]+math.pi) % (2*math.pi)-math.pi
        eligible = walls[f['side'], f['y'], f['x']] & ~viewed[f['side'], f['y'], f['x']]
        eligible &= ((distances > .42) & (distances <= self.maximum_camera_range_m)
                     & (incidence > .6) & (np.abs(bearing) > fov/2+.08))
        ids = np.flatnonzero(eligible)
        ids = ids[visible(data, (camera[1]-origin[1])/resolution,
                          (camera[0]-origin[0])/resolution, f['py'][ids], f['px'][ids])]
        choices = []
        odom_angle = odom_camera[2]-camera[2]
        c, s = math.cos(odom_angle), math.sin(odom_angle)
        for degrees in (-100, -90, -75, -60, -45, 45, 60, 75, 90, 100):
            turn = math.radians(degrees)
            selected = ids[np.abs((bearing[ids]-turn+math.pi) % (2*math.pi)-math.pi) < fov/2-.12]
            length = len(selected)*resolution
            if length < .6:
                continue
            middle = selected[np.argmin(abs(wrap_array(bearing[selected]-turn)))]
            point = np.array([origin[0]+f['px'][middle]*resolution,
                              origin[1]+f['py'][middle]*resolution])
            delta = point-np.asarray(camera[:2])
            anchor = np.asarray(odom_camera[:2])+np.array([c*delta[0]-s*delta[1], s*delta[0]+c*delta[1]])
            odom_yaw = wrap(odom_camera[2]+turn)
            if any(math.dist(anchor, old['wall_anchor_odom']) < 1.2
                   and abs(wrap(odom_yaw-old['odom_yaw'])) < math.radians(55)
                   for old in self.history):
                continue
            # Include the likely reorientation when resuming the original path.
            estimated = 2*abs(turn)/.45+1.2
            allowance = min(self.maximum_total_sim_s, 6+.07*elapsed)
            if self.spent+estimated > allowance or estimated > remaining-45:
                continue
            angular_gain = float(np.sum(resolution*incidence[selected]/distances[selected]))
            choices.append((angular_gain/estimated, dict(turn_rad=turn,
                gain_surface_m=float(length), angular_gain_rad=angular_gain,
                estimated_sim_s=estimated, odom_camera=list(odom_camera), odom_yaw=odom_yaw,
                wall_anchor_odom=anchor.tolist(), map_surface=point.tolist(),
                face_count=len(selected), range_min_m=float(distances[selected].min()),
                range_max_m=float(distances[selected].max()), source='observed_unviewed_wall')))
        return max(choices, key=lambda pair: pair[0])[1] if choices else None

    def finish(self, now, reason):
        active = self.active
        if active is None:
            return
        super().finish(now, reason)
        charge = max(active['actual_sim_s'], active['estimated_sim_s'])
        self.spent += charge-active['actual_sim_s']
        active['budget_charged_s'] = charge

    def report(self):
        result = super().report()
        result.update(mode='wall_transit_v152', budget_charged_s=self.spent,
            spent_sim_s=sum(row.get('actual_sim_s', 0.) for row in self.history),
            minimum_interval_s=25, candidate_recheck_s=2,
            wall_classification='existing SLAM observed-floor heuristic',
            ground_truth_input=False)
        return result


def wrap_array(a):
    return (a+math.pi) % (2*math.pi)-math.pi
