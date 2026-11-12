"""Bound camera inspection turns by new visible evidence, never target truth."""
import math
import numpy as np
from osp_mission.edge_policy import boundary_faces, visible


def select(data, resolution, origin, camera, viewed, focal, fov,
           maximum=2*math.pi/3, minimum_gain=.12):
    """Shortest signed sweep retaining 90% of the best bounded unseen-face gain.

    Surfaces already in the current image are excluded: the caller waits for
    processed camera frames before moving on. Incidence and pixel-size limits
    match the conservative 8 cm marker prior; no observation credit is created.
    """
    if not (math.isfinite(focal) and focal>0 and 0<fov<math.pi): return 0.
    if not (math.isfinite(maximum) and maximum>0): return 0.
    maximum=min(maximum,2*math.pi/3)
    f=boundary_faces(data,resolution)
    if not len(f['side']): return 0.
    if viewed.shape!=(4,)+data.shape: raise ValueError('Four boundary-face states required')
    dy=(camera[1]-origin[1])/resolution-f['py']
    dx=(camera[0]-origin[0])/resolution-f['px']
    lengths=np.hypot(dy,dx)*resolution
    incidence=(dy*f['ny']+dx*f['nx'])/np.maximum(lengths/resolution,1e-9)
    bearing=(np.arctan2(-dy,-dx)-camera[2]+math.pi)%(2*math.pi)-math.pi
    eligible=~viewed[f['side'],f['y'],f['x']]
    eligible &= (lengths>.25)&(incidence>.6)&(lengths<min(8.,.85*focal*.08/18)*incidence)
    eligible &= np.abs(bearing)>fov/2  # already visible faces need dwell, not rotation
    ids=np.flatnonzero(eligible)
    if not len(ids): return 0.
    clear=visible(data,(camera[1]-origin[1])/resolution,(camera[0]-origin[0])/resolution,
                  f['py'][ids],f['px'][ids])
    bearings=bearing[ids[clear]]
    if not len(bearings): return 0.
    # A 15-degree grid also includes the exact requested angular cap.
    angles=np.unique(np.r_[np.arange(math.pi/12,maximum,math.pi/12),maximum])
    candidates=[]
    for sign in (1.,-1.):
        relative=sign*bearings
        for angle in angles:
            gain=float(np.count_nonzero((relative>fov/2)&(relative<angle+fov/2)))*resolution
            candidates.append((gain,sign*float(angle)))
    best=max(g for g,a in candidates)
    if best<minimum_gain: return 0.
    return min((a for g,a in candidates if g>=max(minimum_gain,.9*best)),key=abs)
