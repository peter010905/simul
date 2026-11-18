"""Incremental finite-support surface normals with the same full-grid result."""

import numpy as np

from osp_mission.directional_policy import normals


class AxisNormalCache:
    def __init__(self):
        self.occupied=None;self.resolution=None;self.vectors=None
        self.full_updates=0;self.local_updates=0

    def update(self,data,resolution):
        occupied=data>50
        if self.occupied is None or self.occupied.shape!=data.shape or self.resolution!=resolution:
            self.vectors=normals(data,resolution);self.full_updates+=1
        else:
            changed=np.argwhere(occupied!=self.occupied)
            if not len(changed):return self.vectors
            # scipy Gaussian's default finite support, plus one cell for the
            # gradient of its output. A second halo gives the affected output
            # region the same input support as a full-map calculation.
            radius=int(4*max(.7,.20/resolution)+.5)+1
            low=np.maximum(0,changed.min(axis=0)-radius)
            high=np.minimum(data.shape,changed.max(axis=0)+radius+1)
            crop_low=np.maximum(0,low-radius);crop_high=np.minimum(data.shape,high+radius)
            crop=tuple(slice(int(a),int(b)) for a,b in zip(crop_low,crop_high))
            fresh=normals(data[crop],resolution)
            output=tuple(slice(int(a),int(b)) for a,b in zip(low,high))
            inner=tuple(slice(int(a),int(b)) for a,b in zip(low-crop_low,high-crop_low))
            for old,new in zip(self.vectors,fresh):old[output]=new[inner]
            self.local_updates+=1
        self.occupied=occupied;self.resolution=resolution
        return self.vectors
