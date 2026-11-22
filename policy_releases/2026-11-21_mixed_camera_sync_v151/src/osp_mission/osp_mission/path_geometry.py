"""Continuous path clearance from observed occupancy, without hidden geometry."""

import math

import numpy as np


def segment_clear(data,resolution,start,end,clearance=.32):
    """Check the whole swept line against occupied and unobserved cell centres.

    Coordinates are in grid-centre units, where (0, 0) is the first cell's
    centre. The clearance has a 2 cm margin over the local 30 cm stop radius.
    """
    a=np.asarray(start,dtype=float);b=np.asarray(end,dtype=float)
    if not np.isfinite(a).all() or not np.isfinite(b).all():return False
    radius=clearance/resolution
    if (np.minimum(a,b)-radius<-.5).any() or (np.maximum(a,b)+radius>np.asarray(data.shape)-.5).any():
        return False
    y0=max(0,math.floor(min(a[0],b[0])-radius))
    y1=min(data.shape[0],math.ceil(max(a[0],b[0])+radius)+1)
    x0=max(0,math.floor(min(a[1],b[1])-radius))
    x1=min(data.shape[1],math.ceil(max(a[1],b[1])+radius)+1)
    if y0>=y1 or x0>=x1:return False
    ys,xs=np.where(data[y0:y1,x0:x1]!=0)
    if not len(ys):return True
    delta=b-a;length_squared=float(delta@delta)
    points=np.column_stack((ys+y0,xs+x0))-a
    fraction=np.clip(points@delta/max(length_squared,1e-20),0,1)
    difference=points-fraction[:,None]*delta
    distances_squared=np.sum(difference*difference,axis=1)*resolution**2
    return bool(np.all(distances_squared>clearance**2+1e-12))
