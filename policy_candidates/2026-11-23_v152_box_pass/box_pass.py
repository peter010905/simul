"""Select an extra camera view of a compact obstacle from the current route."""
import math

import numpy as np
from scipy import ndimage

from osp_mission.edge_policy import boundary_faces, visible
from osp_mission.near_pass_inspection import NearPassInspection
from osp_mission.policy import wrap


class BoxPassInspection(NearPassInspection):
    maximum_camera_range_m = 1.75
    maximum_turn_rad = math.radians(95)
    maximum_looks = 6
    maximum_total_sim_s = 30.
    minimum_surface_length_m = .28

    def choose(self, data, resolution, origin, camera, robot, viewed, focal, fov,
               odom_camera, now, elapsed, remaining, protected=False):
        if (protected or self.active is not None or elapsed < 10. or remaining < 60.
                or len(self.history) >= self.maximum_looks or now-self.last_started < 18.
                or not np.isfinite([resolution, *origin, *camera, *robot, focal, fov,
                                    *odom_camera, now, elapsed, remaining]).all()
                or resolution <= 0 or focal <= 0 or not 0 < fov < math.pi):
            return None

        col, row = np.floor((np.asarray(robot)-origin)/resolution).astype(int)
        if not (0 <= row < data.shape[0] and 0 <= col < data.shape[1]):
            return None
        span = math.ceil(2.9/resolution)
        top, left = max(0, row-span), max(0, col-span)
        rows = slice(top, row+span+1)
        cols = slice(left, col+span+1)
        local = data[rows, cols]
        clearance = ndimage.distance_transform_edt(np.pad(local == 0, 1))[row-top+1, col-left+1]
        if clearance*resolution < .32:
            return None
        labels, _ = ndimage.label(local > 50, structure=np.ones((3, 3)))
        compact = set()
        for ident, bounds in enumerate(ndimage.find_objects(labels), 1):
            y, x = bounds
            if y.start == 0 or x.start == 0 or y.stop == local.shape[0] or x.stop == local.shape[1]:
                continue
            width, height = (x.stop-x.start)*resolution, (y.stop-y.start)*resolution
            if .32 <= max(width, height) <= 1.15 and min(width, height) >= .19:
                compact.add(ident)
        if not compact:
            return None

        faces = boundary_faces(local, resolution)
        ids = labels[faces['y'], faces['x']]
        unseen = ~viewed[faces['side'], faces['y']+top, faces['x']+left]
        cx = (camera[0]-origin[0])/resolution-left
        cy = (camera[1]-origin[1])/resolution-top
        dx, dy = faces['px']-cx, faces['py']-cy
        ranges = np.hypot(dx, dy)*resolution
        incidence = -(dx*faces['nx']+dy*faces['ny'])/np.maximum(np.hypot(dx, dy), 1e-9)
        bearing = (np.arctan2(dy, dx)-camera[2]+math.pi) % (2*math.pi)-math.pi
        usable = np.flatnonzero(
            np.isin(ids, list(compact)) & unseen & (ranges >= .45) &
            (ranges <= self.maximum_camera_range_m) & (incidence >= .68))
        usable = usable[visible(local, cy, cx, faces['py'][usable], faces['px'][usable])]
        if not len(usable):
            return None

        best = None
        turn_min = max(.35, fov/2-.13)
        odom_angle = odom_camera[2]-camera[2]
        c, s = math.cos(odom_angle), math.sin(odom_angle)
        for ident in compact:
            for side in range(4):
                selected = usable[(ids[usable] == ident) & (faces['side'][usable] == side)]
                if len(selected)*resolution < self.minimum_surface_length_m:
                    continue
                turn = float(np.median(bearing[selected]))
                if not turn_min <= abs(turn) <= self.maximum_turn_rad:
                    continue
                selected = selected[abs(bearing[selected]-turn) <= fov/2-.12]
                length = len(selected)*resolution
                if length < self.minimum_surface_length_m:
                    continue
                x = float(np.mean(dx[selected])*resolution)
                y = float(np.mean(dy[selected])*resolution)
                anchor = [odom_camera[0]+c*x-s*y, odom_camera[1]+s*x+c*y]
                normal = wrap(math.atan2(float(np.mean(faces['ny'][selected])),
                                         float(np.mean(faces['nx'][selected])))+odom_angle)
                if any(math.dist(anchor, old['face_odom']) < .6 and
                       abs(wrap(normal-old['normal_odom'])) < .8 for old in self.history):
                    continue
                estimated = abs(turn)/.42+2.5
                allowance = min(self.maximum_total_sim_s, 6.+.07*elapsed)
                if self.spent+estimated > allowance:
                    continue
                score = float(np.sum(resolution*incidence[selected]/ranges[selected]))/estimated
                choice = dict(source='box_pass', odom_camera=list(odom_camera),
                              turn_rad=turn, odom_yaw=wrap(odom_camera[2]+turn),
                              face_odom=anchor, normal_odom=normal, gain_surface_m=length,
                              estimated_sim_s=estimated)
                if best is None or score > best[0]:
                    best = score, choice
        return best[1] if best else None
