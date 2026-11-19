import math
import unittest
import numpy as np
from osp_mission.corner_pair import rate_corner, walking_distance


class Grid:
    def __init__(self,clear=True):
        self.clear=clear

    def line_clear(self,start,end):
        return self.clear


class CornerTest(unittest.TestCase):
    def rate(self,seen=(),clear=True,half_fov=.45):
        return rate_corner(Grid(clear),np.array([1.,1.]),np.array([.3,0.]),np.array([0.,.3]),
                           np.array([0.,1.]),0.,seen,0.,1.,half_fov)

    def test_two_readable_unseen_faces(self):
        score,aim,yaw,gain=self.rate()
        self.assertEqual(gain,2)
        np.testing.assert_allclose(aim,[.15,.15])
        self.assertAlmostEqual(yaw,-3*math.pi/4)

    def test_blocked_seen_or_outside_frame(self):
        for args in [dict(seen=[math.pi/2]),dict(clear=False),dict(half_fov=.03)]:
            _,aim,_,gain=self.rate(**args)
            self.assertEqual(gain,1)
            np.testing.assert_allclose(aim,[.3,0.])

    def test_route_around_obstacle(self):
        mask=np.ones((9,9),bool)
        mask[1:8,4]=False
        steps=walking_distance(mask,(4,3),.1)
        self.assertEqual(steps[4,5],10)
        self.assertNotIn((4,5),walking_distance(mask,(4,3),.1,.8))


if __name__=='__main__':unittest.main()
