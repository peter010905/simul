"""Camera-confirmed directional coverage for the live ROS search planner."""

import math

import numpy as np

from osp_mission.directional_policy import direction_bins, normals
from osp_mission.coverage_resize import remap
from osp_mission.surface_hits import match


class SurfaceCoverage:
    """Count independent camera frames, retaining opposite faces separately."""

    def __init__(self, directional=True):
        self.directional = directional
        self.geometry = None
        self.viewed = None
        self.counts = None
        self.last_stamp = -float('inf')
        self.normal_occupancy = None
        self.normal_vectors = None
        self.data = None

    def grid(self, data, resolution, origin):
        geometry = (data.shape, resolution, tuple(origin))
        if self.geometry != geometry:
            if self.geometry is None:
                self.viewed = np.zeros((8,) + data.shape if self.directional else data.shape, dtype=bool)
                self.counts = np.zeros_like(self.viewed, dtype=np.uint8)
            else:
                self.viewed = remap(self.viewed, self.data, self.geometry, data, resolution, origin)
                self.counts = remap(self.counts, self.data, self.geometry, data, resolution, origin)
            self.geometry = geometry
            self.normal_occupancy = None
        self.data = data.copy()
        return self.viewed

    def observe(self, data, resolution, origin, camera, endpoints, stamp,
                focal, horizontal_fov):
        self.grid(data, resolution, origin)
        if stamp - self.last_stamp < .15 - 1e-9:
            return
        if not (math.isfinite(stamp) and math.isfinite(focal) and focal > 0
                and 0 < horizontal_fov < math.pi):
            return
        self.last_stamp = stamp
        points, ys, xs = match(data, resolution, origin, endpoints)
        if not len(points):
            return
        occupied = data > 50
        if self.normal_occupancy is None or not np.array_equal(
                self.normal_occupancy, occupied):
            self.normal_occupancy = occupied.copy()
            if self.directional:
                self.normal_vectors = normals(data, resolution)
            else:
                from osp_mission.surface_endpoint_policy import normals as surface_normals
                self.normal_vectors = surface_normals(data, resolution)
        ny, nx = self.normal_vectors
        dx = camera[0] - points[:, 0]
        dy = camera[1] - points[:, 1]
        distance = np.hypot(dx, dy)
        angle = np.arctan2(-dy, -dx)
        relative = (angle - camera[2] + math.pi) % (2 * math.pi) - math.pi
        incidence = (dx * nx[ys, xs] + dy * ny[ys, xs]) / np.maximum(distance, 1e-9)
        if self.directional:
            incidence = np.abs(incidence)
        reliable = ((data[ys, xs] > 50) & (distance > .25)
                    & (np.abs(relative) < horizontal_fov / 2)
                    & (incidence > .6)
                    & (distance < .85 * focal * .12 * incidence / 18)
                    & (distance * np.cos(relative) < 8.))
        bins = direction_bins(dy, dx) if self.directional else np.zeros(len(dy),dtype=int)
        # LiDAR and camera are offset on the robot. A LiDAR hit can be hidden
        # from the camera by a nearby corner, so check the observed camera ray.
        indices = np.flatnonzero(reliable)
        if len(indices):
            camera_x = (camera[0] - origin[0]) / resolution - .5
            camera_y = (camera[1] - origin[1]) / resolution - .5
            lengths = np.hypot(ys[indices] - camera_y, xs[indices] - camera_x)
            stop = np.maximum(0., 1. - 1.01 / np.maximum(lengths, 1e-9))
            fractions = stop[:, None] * np.linspace(
                0., 1., max(4, int(np.max(lengths) * 2) + 1))
            ray_y = np.rint(camera_y + (ys[indices, None] - camera_y) * fractions).astype(int)
            ray_x = np.rint(camera_x + (xs[indices, None] - camera_x) * fractions).astype(int)
            ray_inside = ((ray_y >= 0) & (ray_y < data.shape[0])
                          & (ray_x >= 0) & (ray_x < data.shape[1]))
            visible = np.all(ray_inside & (data[
                ray_y.clip(0, data.shape[0] - 1),
                ray_x.clip(0, data.shape[1] - 1)] == 0), axis=1)
            reliable[indices] &= visible
        cells = np.unique(np.stack(
            (bins[reliable], ys[reliable], xs[reliable]), axis=1), axis=0)
        if not len(cells):
            return
        b, y, x = cells.T
        if not self.directional:
            self.counts[y,x] = np.minimum(self.counts[y,x]+1,3)
            self.viewed[y,x] = self.counts[y,x]>=3
            return
        self.counts[b, y, x] = np.minimum(self.counts[b, y, x] + 1, 3)
        confirmed = self.counts[b, y, x] >= 3
        b, y, x = b[confirmed], y[confirmed], x[confirmed]
        for offset in (-1, 0, 1):
            self.viewed[(b + offset) % 8, y, x] = True


class DirectionalCoverage(SurfaceCoverage):
    """Backward-compatible directional observation memory."""

    def __init__(self):
        super().__init__(directional=True)
