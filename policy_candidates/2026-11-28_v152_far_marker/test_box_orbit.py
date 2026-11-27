"""Unit checks for the .70 box-orbit geometry (run inside the staged workspace's Python path)."""
import math
import unittest

import numpy as np

from osp_mission.box_orbit import BoxOrbit, GAP_DONE, largest_gap


class FakeGrid:
    """5 cm grid, origin at (-3, -3), a 0.6 m box at the origin, everything else free."""
    resolution = .05

    def __init__(self):
        self.cells = np.zeros((120, 120), dtype=np.int16)
        self.cells[54:66, 54:66] = 100

    def index(self, x, y):
        return math.floor((y+3)/self.resolution), math.floor((x+3)/self.resolution)

    def inside(self, row, col):
        return 0 <= row < 120 and 0 <= col < 120

    def line_clear(self, a, b, allow_unknown=False):
        a, b = np.asarray(a), np.asarray(b)
        for t in np.linspace(0, 1, 60)[:-2]:
            r, c = self.index(*(a+(b-a)*t))
            if not self.inside(r, c) or self.cells[r, c] > 25:
                return False
        return True


class BoxOrbitTest(unittest.TestCase):
    def test_largest_gap(self):
        self.assertAlmostEqual(largest_gap([])[0], math.tau)
        gap, middle = largest_gap([0., math.pi/2])
        self.assertAlmostEqual(gap, 1.5*math.pi)
        self.assertAlmostEqual(math.remainder(middle - 1.25*math.pi, math.tau), 0.)

    def test_one_view_then_far_side_target_then_done(self):
        grid, orbit, identity = FakeGrid(), BoxOrbit(), (0., 0., 0.)
        orbit.observe([(0., 0.)], grid, (1.2, 0., math.pi), identity)       # looking at the +x side
        item = orbit.boxes[0]
        self.assertEqual(len(item['bearings']), 1)
        orbit.observe([(0., 0.)], grid, (1.2, 0., 0.), identity)            # facing away: not a view
        self.assertEqual(len(item['bearings']), 1)
        nav = np.ones_like(grid.cells, dtype=bool)
        distances = np.where(grid.cells > 25, np.inf, 1.).astype(float)
        clearance = np.full(grid.cells.shape, 1.)
        goal = orbit.choose([(0., 0.)], set(), None, grid, grid, nav, distances, clearance,
                            identity, lambda g: False, (1.2, 0., math.pi), 10.)
        self.assertIsNotNone(goal)
        self.assertLess(goal['x'], -.5)                                      # the far (-x) side
        orbit.dispatched(goal, 10.)
        for angle in (math.pi/2, math.pi, -math.pi/2):
            p = (1.2*math.cos(angle), 1.2*math.sin(angle))
            orbit.observe([(0., 0.)], grid, (*p, angle+math.pi), identity)
        self.assertLessEqual(orbit.gap(item)[0], GAP_DONE)
        self.assertIsNone(orbit.choose([(0., 0.)], set(), None, grid, grid, nav, distances, clearance,
                                       identity, lambda g: False, (1.2, 0., math.pi), 20.))

    def test_completed_box_is_skipped(self):
        grid, orbit, identity = FakeGrid(), BoxOrbit(), (0., 0., 0.)
        orbit.observe([(0., 0.)], grid, (1.2, 0., math.pi), identity)
        self.assertFalse(orbit.pending(orbit.boxes[0], {(0., 0.)}, 5.))


if __name__ == '__main__':
    unittest.main()
