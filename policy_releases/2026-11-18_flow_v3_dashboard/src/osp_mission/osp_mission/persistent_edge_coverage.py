"""Keep verified camera observations in metric coordinates across SLAM rasters."""

import numpy as np
from scipy.spatial import cKDTree

from osp_mission.edge_policy import EdgeCoverage


class PersistentEdgeCoverage(EdgeCoverage):
    """Reassociate confirmed faces, requiring nearby position and matching normal.

    Anchors are created only when actual independent camera frames confirm a
    new face. Reprojection never creates another anchor, so repeated map
    updates cannot walk an observation along a wall or onto its opposite side.
    """

    def __init__(self):
        super().__init__()
        self.anchors = np.empty((0, 4), dtype=float)  # world x,y and normal x,y
        self.anchor_keys = set()
        self.reassociated_faces = 0

    def grid(self, data, resolution, origin):
        changed = (self.geometry != (data.shape, resolution, tuple(origin))
                   or self.data is None or not np.array_equal(data, self.data))
        super().grid(data, resolution, origin)
        if changed:
            # Confirmation comes only from the original metric anchors. Do not
            # also retain the base class's iterative raster-to-raster transfer.
            self.viewed.fill(False)
            self.counts[self.counts >= 3] = 0
        if not changed or not len(self.anchors) or not len(self.faces['side']):
            return self.viewed
        faces = self.faces
        points = np.column_stack((origin[0] + resolution * faces['px'],
                                  origin[1] + resolution * faces['py']))
        normals = np.column_stack((faces['nx'], faces['ny']))
        distance, index = cKDTree(points).query(self.anchors[:, :2], k=min(8, len(points)))
        if distance.ndim == 1:
            distance, index = distance[:, None], index[:, None]
        alignment = np.sum(self.anchors[:, None, 2:] * normals[index], axis=2)
        # At 5 cm resolution this permits at most 7.5 cm of local SLAM drift.
        eligible = ((distance <= min(.075, 1.5 * resolution)) & (alignment >= .8))
        usable = eligible.any(axis=1)
        nearest = np.argmin(np.where(eligible, distance, np.inf), axis=1)
        # One anchor can credit only one current face, never a dilated patch.
        matched = np.unique(index[np.arange(len(index))[usable], nearest[usable]])
        side, y, x = (faces[name][matched] for name in ('side', 'y', 'x'))
        self.reassociated_faces += int((~self.viewed[side, y, x]).sum())
        self.viewed[side, y, x] = True
        self.counts[side, y, x] = 3
        return self.viewed

    def observe(self, data, resolution, origin, camera, endpoints, stamp,
                focal, horizontal_fov):
        self.grid(data, resolution, origin)
        faces = self.faces
        side, y, x = (faces[name] for name in ('side', 'y', 'x'))
        before = self.viewed[side, y, x].copy()
        super().observe(data, resolution, origin, camera, endpoints, stamp,
                        focal, horizontal_fov)
        confirmed = self.viewed[side, y, x] & ~before
        if not confirmed.any():
            return
        new = np.column_stack((origin[0] + resolution * faces['px'][confirmed],
                               origin[1] + resolution * faces['py'][confirmed],
                               faces['nx'][confirmed], faces['ny'][confirmed]))
        additions = []
        for anchor in new:
            key = tuple(np.rint(anchor / [.01, .01, .1, .1]).astype(int))
            if key not in self.anchor_keys:
                self.anchor_keys.add(key)
                additions.append(anchor)
        if additions:
            self.anchors = np.vstack((self.anchors, additions))
