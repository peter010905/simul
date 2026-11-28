""".74: a lap whose progress does not rise for 90 s of lap dispatching ends."""
import unittest

from osp_mission.wall_lap import WallLap


class LapStallTest(unittest.TestCase):
    def test_stalled_lap_finishes(self):
        lap = WallLap()
        lap.entry = (0., 0.)
        lap.perimeter_m = 40.
        lap.last_was_lap = True
        t = 10.
        while lap.phase == 'perimeter' and t < 200.:
            lap.expire(t, 600.)
            t += 4.
        self.assertEqual(lap.reason, 'perimeter_no_progress')
        self.assertLess(t, 115.)
        self.assertGreater(t, 90.)

    def test_progressing_lap_continues(self):
        lap = WallLap()
        lap.entry = (0., 0.)
        lap.perimeter_m = 40.
        lap.last_was_lap = True
        for k in range(30):
            lap.path = 2.*k
            lap.expire(10.+4.*k, 600.)
        self.assertEqual(lap.phase, 'perimeter')


if __name__ == '__main__':
    unittest.main()
