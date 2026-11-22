"""A local exception in the exploration graph, never in sensor or costmap data."""
import math

import numpy as np


def validate_radius(radius):
    radius = float(radius)
    if not math.isfinite(radius) or not 0. <= radius <= .30:
        raise ValueError('Planner start exception must be between 0 and 0.30 m')
    return radius


def open_start_region(safe, resolution, origin, robot, radius):
    """Connect the current pose through a disk in a private planning mask.

    This is not a collision-free path certificate. Nav2 must plan and execute
    against its original map and scan. Goal validity also uses original data.
    """
    radius = validate_radius(radius)
    result = safe.copy()
    if radius == 0.:
        return result
    if resolution <= 0. or not np.isfinite([resolution, *origin, *robot]).all():
        raise ValueError('Invalid planning grid or robot pose')
    y, x = np.ogrid[:safe.shape[0], :safe.shape[1]]
    distance_squared = ((origin[0]+(x+.5)*resolution-robot[0])**2
                        +(origin[1]+(y+.5)*resolution-robot[1])**2)
    result[distance_squared <= radius**2+1e-12] = True
    return result
