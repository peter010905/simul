"""The .71 bearing look reaches 3 m along the ray and crosses unknown cells."""
import math
import unittest

import numpy as np

from osp_mission import bearing_advance as ba


class Grid:
    resolution = .05

    def __init__(self):
        self.cells = np.full((200, 200), -1, dtype=np.int16)    # unknown everywhere
        self.cells[90:110, 0:200] = 0                             # a measured free corridor along +x

    def index(self, x, y):
        return math.floor(y/self.resolution)+100, math.floor(x/self.resolution)+100

    def inside(self, r, c):
        return 0 <= r < 200 and 0 <= c < 200

    def line_clear(self, a, b, allow_unknown=False):
        for t in np.linspace(0, 1, 80)[:-2]:
            r, c = self.index(*(np.asarray(a)+(np.asarray(b)-np.asarray(a))*t))
            v = self.cells[r, c]
            if v > 25 or (v < 0 and not allow_unknown):
                return False
        return True


class FarMarkerTest(unittest.TestCase):
    def test_far_endpoint_for_small_print(self):
        grid = Grid()
        adv = ba.BearingAdvance()
        for k in range(3):
            adv.observe(dict(id=7, stamp=10.+.2*k, origin_odom=[0., 0.], yaw_odom=0., pixels=11.), 10.+.2*k+.1)
        self.assertIn(7, adv.supported)
        safe = grid.cells == 0
        nav = np.ones_like(safe)
        yy, xx = np.mgrid[0:200, 0:200]
        distances = np.where(safe, np.hypot((xx-100)*.05, (yy-100)*.05), np.inf)
        goal = adv.choose(grid, grid, safe, nav, distances, (0., 0., 0.), (0., 0., 0.), set(), 11., 300.,
                          lambda g: False)
        self.assertIsNotNone(goal)
        self.assertGreater(goal['x'], 2.5)


if __name__ == '__main__':
    unittest.main()
