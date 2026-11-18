"""Reachable frontier candidates with robot clearance."""
import math
import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt, label, uniform_filter


def reachable_gain(data, resolution, reachable):
    """Reward open edges of reachable free space, excluding unknown behind walls.

    ``reachable`` includes the caller's robot clearance and connectivity checks.
    Normalize local frontier length so the feature is stable across map scales.
    """
    edge = reachable & binary_dilation(data < 0)
    groups, count = label(edge, structure=np.ones((3, 3)))
    if count:
        sizes = np.bincount(groups.ravel()) * resolution
        large = sizes >= .25
        large[0] = False
        edge &= large[groups]
    size = max(3, int(3 / resolution) | 1)
    length = uniform_filter(edge.astype(float), size=size, mode='constant') * size**2 * resolution
    return np.minimum(1., length / 3.)


def candidates(data, resolution, origin, robot, excluded=()):
    free=data==0; unknown=data<0
    safe=free & (distance_transform_edt(free)*resolution>.26)
    rx=int((robot[0]-origin[0])/resolution); ry=int((robot[1]-origin[1])/resolution)
    if not(0<=ry<data.shape[0] and 0<=rx<data.shape[1]): return []
    # Connectivity follows free space; clearance is enforced on destinations.
    regions,_=label(free); region=regions[ry,rx]
    if not region: return []
    edge=free & binary_dilation(unknown) & (regions==region)
    groups,count=label(edge,structure=np.ones((3,3)))
    result=[]
    rows,cols=np.indices(data.shape)
    for i in range(1,count+1):
        ys,xs=np.where(groups==i)
        if len(xs)*resolution<.25: continue
        cy,cx=float(ys.mean()),float(xs.mean())
        # Stand back from the unknown boundary, inside observed free space.
        window=safe & (regions==region) & ((rows-cy)**2+(cols-cx)**2 < (1./resolution)**2)
        wy,wx=np.where(window)
        if not len(wx): continue
        j=np.argmin((wx-cx)**2+(wy-cy)**2)
        x,y=origin[0]+(wx[j]+.5)*resolution,origin[1]+(wy[j]+.5)*resolution
        if any(math.hypot(x-a,y-b)<.75 for a,b in excluded): continue
        distance=math.hypot(x-robot[0],y-robot[1])
        result.append((len(xs)*resolution/(1+distance),x,y,math.atan2(cy-wy[j],cx-wx[j])))
    return sorted(result,reverse=True)
