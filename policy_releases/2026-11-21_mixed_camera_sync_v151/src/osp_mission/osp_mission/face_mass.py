"""Cap observed connected face length without using obstacle or marker labels."""

import math
import numpy as np
from scipy.ndimage import label


def weights(data, resolution, cap=1.5):
    if not math.isfinite(cap) or cap<=0:
        raise ValueError('Face-length cap must be positive and finite')
    occupied=data>50;free=data==0
    adjacent=np.zeros((4,)+data.shape,dtype=bool)
    adjacent[0,1:]=free[:-1];adjacent[1,:,:-1]=free[:,1:]
    adjacent[2,:-1]=free[1:];adjacent[3,:,1:]=free[:,:-1]
    result=np.zeros((4,)+data.shape)
    for side in range(4):
        groups,count=label(adjacent[side]&occupied,structure=np.ones((3,3)))
        lengths=np.bincount(groups.ravel(),minlength=count+1)*resolution
        value=np.minimum(1.,cap/np.maximum(lengths,1e-12));value[0]=0.
        result[side]=value[groups]
    return result


def reweight(candidates, mass, resolution):
    result=[]
    for candidate in candidates:
        item=candidate.copy();item['features']=candidate['features'].copy()
        faces=candidate['visible_faces']
        item['features'][0]=float(mass[faces[:,0],faces[:,1],faces[:,2]].sum())*resolution
        result.append(item)
    return result
