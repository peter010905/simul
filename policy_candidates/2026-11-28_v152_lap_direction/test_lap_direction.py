""".72: lap progress counts credited angle around the floor centre in one direction only."""
import math
import unittest

from osp_mission.wall_lap import WallLap


class LapDirectionTest(unittest.TestCase):
    def credit(self, lap, x, y, t):
        lap.goal = dict(lap_pose_odom=[x, y], lap_normal_odom=0.)
        lap.active = True
        lap.position(x, y, t)

    def test_backward_credit_adds_nothing(self):
        lap = WallLap()
        lap.centre_odom = [0., 0.]
        lap.sense = 1
        self.credit(lap, 3., 0., 1.)                                    # entry, 0 deg
        self.credit(lap, 3*math.cos(.5), 3*math.sin(.5), 2.)            # +28.6 deg
        self.credit(lap, 3*math.cos(.2), 3*math.sin(.2), 3.)            # back 17 deg: no credit
        self.credit(lap, 3*math.cos(1.), 3*math.sin(1.), 4.)            # +45.8 deg from the last
        self.assertAlmostEqual(lap.angular, .5 + .8, places=6)
        self.assertAlmostEqual(lap.progress(), 1.3/(2*math.pi), places=6)


if __name__ == '__main__':
    unittest.main()
