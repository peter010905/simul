"""Select a short camera glance at uninspected visible faces during travel."""

import math
import numpy as np

from osp_mission.edge_policy import visible
from osp_mission.policy import wrap


def select(data, resolution, origin, camera, faces, viewed, focal, fov, minimum_gain=.75):
    """Return a body heading, using only observed boundary faces and camera data."""
    if not len(faces['side']):
        return None
    dy = (camera[1] - origin[1]) / resolution - faces['py']
    dx = (camera[0] - origin[0]) / resolution - faces['px']
    lengths = np.hypot(dy, dx)
    incidence = (dy * faces['ny'] + dx * faces['nx']) / np.maximum(lengths, 1e-9)
    bearings = np.arctan2(-dy, -dx)
    relative = np.abs((bearings - camera[2] + math.pi) % (2 * math.pi) - math.pi)
    eligible = ~viewed[faces['side'], faces['y'], faces['x']]
    eligible &= (lengths * resolution > .25) & (incidence > .6)
    eligible &= lengths * resolution < min(8., .85 * focal * .12 / 18) * incidence
    # Forward-facing surfaces already receive observations during normal travel.
    eligible &= relative > fov / 2 + .05
    ids = np.flatnonzero(eligible)
    if len(ids) * resolution < minimum_gain:
        return None
    clear = visible(data, (camera[1] - origin[1]) / resolution,
                    (camera[0] - origin[0]) / resolution, faces['py'][ids], faces['px'][ids])
    ids = ids[clear]
    if len(ids) * resolution < minimum_gain:
        return None
    sectors = np.arange(-math.pi, math.pi, math.pi / 12)
    gains = np.sum(np.abs((bearings[ids, None] - sectors + math.pi) % (2 * math.pi) - math.pi)
                   < fov / 2 - .08, axis=0) * resolution
    turns = np.abs((sectors - camera[2] + math.pi) % (2 * math.pi) - math.pi)
    usable = (gains >= minimum_gain) & (turns > fov / 2)
    if not usable.any():
        return None
    score = np.where(usable, gains / (2 * turns / .5 + .8), -np.inf)
    best = int(np.argmax(score))
    return dict(yaw=wrap(float(sectors[best])), gain=float(gains[best]),
                estimated_seconds=float(2 * turns[best] / .5 + .8))
