import math
import unittest

import numpy as np

from osp_mission import _r32_navigation_core as r32
from osp_mission.box_views import BoxViews


class TestBoxViews(unittest.TestCase):
    def setUp(self):
        cells = np.zeros((160,160),dtype=np.int8)
        cells[74:87,74:87] = 100
        self.grid = r32.Grid(cells, .05, (-4.,-4.,0.))
        self.pose=(-1.3,0.,0.)
        self.memory=BoxViews()

    def choose(self, now, cost=None, targets=None, veto=None):
        return self.memory.choose(self.grid,cost or self.grid,self.pose,(0,0,0),
                                  now,targets or {},veto or (lambda goal: False))

    def test_different_face_after_dispatch(self):
        first=self.choose(20.)
        self.assertIsNotNone(first)
        self.memory.dispatched(first,20.)
        self.assertIsNone(self.choose(22.))
        second=self.choose(32.)
        self.assertIsNotNone(second)
        self.assertLessEqual(math.cos(second['face_angle_odom']-first['face_angle_odom']),.70)

    def test_costmap_and_failed_goals(self):
        blocked=r32.Grid(np.full(self.grid.cells.shape,100,dtype=np.int8),.05,self.grid.origin)
        self.assertIsNone(self.choose(20.,cost=blocked))
        self.assertIsNone(self.choose(20.,veto=lambda goal: True))
        self.assertIsNone(self.choose(20.,targets={32:(0.,0.)}))

    def test_long_wall_is_not_a_box(self):
        self.grid.cells[:]=0
        self.grid.cells[:,80]=100
        self.assertIsNone(self.choose(20.))


if __name__=='__main__':
    unittest.main()
