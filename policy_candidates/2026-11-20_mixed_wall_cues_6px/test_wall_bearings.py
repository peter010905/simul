import unittest
from types import SimpleNamespace
import numpy as np
from osp_mission.wall_bearings import WallBearings


class WallCueTests(unittest.TestCase):
    def setUp(self):
        self.memory = WallBearings()
        self.grid = SimpleNamespace(line_clear=lambda a,b: True)
        self.goal = dict(surface_odom=[8.,0.],surface_map=[8.,0.], x=7.,y=0.)
        self.walls = [(3.,self.goal)]

    def glimpse(self, start, x, size):
        for dt in (0.,.2,.4):
            self.memory.observe(start+dt,start+dt,(x,0.),0.,size)

    def choose(self, known=(), left=100):
        return self.memory.choose(self.walls,self.grid,(0.,0.,0.),known,10.,left)

    def test_tiny_needs_independent_view(self):
        self.glimpse(1,0.,6.)
        self.assertIsNone(self.choose())
        self.glimpse(4,.8,6.)
        self.assertTrue(self.choose()['weak_wall'])

    def test_single_frame_and_obstacle_in_line_of_sight(self):
        self.memory.observe(1.,1.,(0.,0.),0.,9.)
        self.assertIsNone(self.choose())
        self.glimpse(2,0.,9.)
        self.grid.line_clear=lambda a,b:False
        self.assertIsNone(self.choose())

    def test_known_marker_time_budget_and_repeat(self):
        self.glimpse(1,0.,8.)
        self.assertIsNone(self.choose(known=[np.array([8.,.12])]))
        self.assertIsNone(self.choose(left=7))
        g=self.choose()
        self.assertIsNotNone(g)
        self.memory.dispatched(g)
        self.assertIsNone(self.choose())

    def test_ambiguous_depth(self):
        self.glimpse(1,0.,9.)
        self.walls.append((5.,dict(surface_odom=[12.,0.],surface_map=[12.,0.],x=11.,y=0.)))
        self.assertIsNone(self.choose())


if __name__ == '__main__':
    unittest.main()
