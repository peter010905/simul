import unittest

import numpy as np

from osp_mission._r32_navigation_core import Grid
from osp_mission.center_wall import CenterWall


class Room(unittest.TestCase):
    def setUp(self):
        data = np.full((80,100), -1, np.int8)
        data[5:-5,5:-5] = 100
        data[6:-6,6:-6] = 0
        data[31:38,42:54] = 100
        data[26:33,42:46] = 100
        self.grid = Grid(data,.1,(0.,0.,0.))
        self.cost = self.grid
        self.pose = (3.5,3.6,0.)
        self.viewed = np.zeros((4,)+data.shape, bool)
        self.policy = CenterWall()

    def decide(self,t):
        return self.policy.decide(self.grid,self.cost,self.pose,(0.,0.,0.),
                                  self.viewed,t,600.,lambda _:False)

    def test_irregular_object_first_then_ordered_wall_goals(self):
        first = self.decide(10.)
        self.assertEqual(first['r32_kind'],'interior_surface')
        self.policy.dispatched(first,10.)
        self.viewed[:,24:40,40:56]=True
        self.assertIsNone(self.decide(20.))
        self.assertEqual(self.policy.phase,'interior')
        wall = self.decide(64.)
        self.assertEqual(wall['r32_kind'],'wall_surface')
        self.policy.dispatched(wall,64.)
        self.pose=(wall['x'],wall['y'],wall['yaw'])
        second=self.decide(75.)
        self.assertEqual(second['r32_kind'],'wall_surface')
        step=np.subtract(second['surface_odom'],wall['surface_odom'])
        tangent=np.array([-np.sin(wall['normal_odom']),np.cos(wall['normal_odom'])])
        self.assertGreater(step@tangent,0)

    def test_two_objects_exhaust_open_sides_before_wall_phase(self):
        self.grid.cells[47:54,62:69]=100
        from osp_mission.edge_policy import DIRECTIONS
        exposed=set()
        for i in range(22):
            elapsed=10.+i*6
            goal=self.decide(elapsed)
            if goal is None or goal['r32_kind']=='wall_surface':
                break
            row,col=self.grid.index(*goal['surface_map'])
            which=1 if col>57 else 0
            side=int(np.argmax([np.cos(np.arctan2(dy,dx)-goal['normal_odom'])
                                for dy,dx in DIRECTIONS]))
            exposed.add((which,side))
            self.policy.dispatched(goal,elapsed)
            self.pose=(goal['x'],goal['y'],goal['yaw'])
            # Ideal camera feedback for this side only; no dispatch credit.
            if which:
                self.viewed[side,45:56,60:71]=True
            else:
                self.viewed[side,24:40,40:56]=True
        self.assertEqual(len(exposed),8)
        self.assertEqual(self.decide(elapsed+40)['r32_kind'],'wall_surface')

    def test_late_untried_interior_first_and_no_unknown_or_blocked_goal(self):
        self.assertEqual(self.decide(332.)['r32_kind'],'interior_surface')
        self.viewed[:,24:40,40:56]=True
        self.assertEqual(self.decide(334.)['r32_kind'],'wall_surface')
        self.assertEqual(self.policy.reason,'no_untried_reachable_interior_view')
        self.cost=Grid(np.full_like(self.grid.cells,-1),.1,(0.,0.,0.))
        self.assertIsNone(self.decide(340.))

    def test_unseen_object_wins_even_in_wall_reserve(self):
        self.assertEqual(self.decide(525.)['r32_kind'],'interior_surface')
        self.viewed[:,24:40,40:56]=True
        self.assertEqual(self.decide(526.)['r32_kind'],'wall_surface')

    def test_new_object_preempts_wall_after_transition(self):
        self.viewed[:,24:40,40:56]=True
        self.decide(20.)
        wall=self.decide(70.)
        self.policy.dispatched(wall,70.)
        self.viewed[:,24:40,40:56]=False
        self.assertEqual(self.decide(79.)['r32_kind'],'interior_surface')
        self.assertIsNone(self.policy.wall_anchor)

    def test_unknown_strip_behind_object_and_straight_perimeter_fragment(self):
        from osp_mission.center_wall import surface_cells
        data=np.full((90,90),-1,np.int8)
        data[10:80,10:80]=0
        data[40:46,40:46]=100
        data[42:44,46:]=-1  # The free-space contour around the object is open.
        data[20:30,9]=100  # A short perimeter return with unknown behind it.
        grid=Grid(data,.1,(0.,0.,0.))
        groups=surface_cells(grid,(3.,3.,0.))
        self.assertTrue(any(k[0]=='interior' and any(c==40 for _,c in v) for k,v in groups.items()))
        self.assertFalse(any(k[0]=='interior' and any(c==9 for _,c in v) for k,v in groups.items()))

    def test_reachable_frontier_preempts_wall_but_blocked_one_does_not(self):
        self.viewed[:,24:40,40:56]=True
        self.decide(20.)
        self.assertEqual(self.decide(70.)['r32_kind'],'wall_surface')
        goal=dict(x=3.,y=3.,features=[0.,3.,2.,0.,0.],distance=2.)
        def choose(t):
            return self.policy.decide(self.grid,self.cost,self.pose,(0.,0.,0.),
                                      self.viewed,t,600.,lambda _:False,[goal])
        self.assertEqual(choose(82.)['r32_kind'],'interior_frontier')
        goal.update(x=0.,y=0.)
        self.assertEqual(choose(530.)['r32_kind'],'wall_surface')

    def test_only_one_reachable_side_is_enough(self):
        cells=np.full_like(self.grid.cells,100)
        cells[20:48,15:39]=0
        self.cost=Grid(cells,.1,(0.,0.,0.))
        g=self.decide(30.)
        self.assertEqual(g['r32_kind'],'interior_surface')
        self.assertLess(g['x'],4.)
        self.assertEqual(self.cost.cells[self.cost.index(g['x'],g['y'])],0)


if __name__=='__main__':
    unittest.main()
