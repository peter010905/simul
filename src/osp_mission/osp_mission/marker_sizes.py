"""Seeded physical marker-size variation, independent of scene geometry."""
import copy
import math

import numpy as np


def face_size_limit(cfg, target):
    """Same carrier-width margin as the existing robust layout generator.

    Size denotes the black marker square; the white backing is 1.25 times wider.
    The world uses a 0.9 m support centered at z=0.45 m for wall-mounted markers.
    """
    if target.get('mount') != 'obstacle':
        return .70
    obstacle=cfg['obstacles'][int(target['carrier_obstacle'])]
    if obstacle['type']=='cylinder':
        width=2*obstacle['radius']
    else:
        angle=target['yaw']-obstacle.get('yaw',0.)
        width=obstacle['sy'] if abs(math.cos(angle))>=abs(math.sin(angle)) else obstacle['sx']
    # Native world marker centers are at z=0.45 m. Fit the white backing
    # above the floor and below the carrier top even for custom large sizes.
    vertical=.95*2*max(0.,min(.45,obstacle['height']-.45))/1.25
    return min(.70*width,vertical,.70)


def randomize_sizes(cfg, seed, minimum=.08, maximum=.30, mode='mixed'):
    """Return a copy; no locations, IDs, obstacles or start poses are changed.

    Mixed assigns shuffled lower/middle/upper thirds of each feasible size range.
    Thus a narrow carrier can have a smaller upper-third marker than a broad wall.
    """
    if not np.isfinite([minimum,maximum]).all() or not .05<=minimum<=maximum<=.8:
        raise ValueError('Marker size limits must satisfy0.05<=minimum<=maximum<=0.8 metres')
    if mode not in ('mixed','uniform'):raise ValueError('Unknown size sampling mode')
    result=copy.deepcopy(cfg);rng=np.random.default_rng(seed)
    n=len(result['targets']);bins=rng.permutation(n)
    for i,target in enumerate(result['targets']):
        upper=min(maximum,face_size_limit(result,target))
        if upper<minimum:raise ValueError('Requested minimum does not fit the marker carrier')
        u=float(rng.random())
        if mode=='mixed':u=(int(bins[i])+u)/max(1,n)
        target['size']=float(minimum+(upper-minimum)*u)
        target['size_sampling_bounds_m']=[float(minimum),float(upper)]
    result['marker_size_randomization']=dict(seed=int(seed),minimum_m=float(minimum),maximum_m=float(maximum),
        mode=mode,unit='metres; black square side length; backing=1.25*size',
        changes_during_episode=False)
    return result
