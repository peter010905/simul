"""Box faces follow a turned box (full outline or an L seen from one corner)."""
import math
import numpy as np
from osp_mission import _r32_navigation_core as r32


def scene(yaw, sx=.8, sy=.6, partial=False, noise=.015, seed=0):
    rng = np.random.default_rng(seed)
    res, n = .05, 160
    cells = np.zeros((n, n), np.int8)
    cells[5, :] = cells[n-6, :] = cells[:, 5] = cells[:, n-6] = 100
    c, s = math.cos(yaw), math.sin(yaw)
    for t in np.linspace(-.5, .5, 80):
        for u, v in ((t*sx, sy/2), (t*sx, -sy/2), (sx/2, t*sy), (-sx/2, t*sy)):
            if partial and (v < -sy/2+1e-6 or u < -sx/2+1e-6):
                continue
            x, y = 4+c*u-s*v+rng.normal(0, noise), 4+s*u+c*v+rng.normal(0, noise)
            cells[int(y/res), int(x/res)] = 100
    return r32.Grid(cells, res, (0., 0., 0.))


def face_error(deg, **kw):
    groups = []
    r32.surface_samples(scene(math.radians(deg), **kw), groups)
    box = [f for f in groups if np.linalg.norm(np.mean([p for p, _ in f], axis=0)-4) < .7]
    normal = box[0][0][1]
    return abs((math.degrees(math.atan2(normal[1], normal[0]))-deg+45) % 90-45)


def test_turned_box_full_and_corner():
    for deg in (20, 36, 45, 70):
        assert face_error(deg) <= 5
        assert face_error(deg, partial=True) <= 5


def test_room_aligned_box_stays_aligned():
    for i in range(10):
        assert face_error(float(np.random.default_rng(i).uniform(-2, 2)), noise=.03, seed=i) <= 6
