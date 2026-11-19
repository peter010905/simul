import math
import unittest

import numpy as np

from osp_mission import _r32_navigation_core as r32
from osp_mission.two_views import TwoViews, similar


class TwoViewTest(unittest.TestCase):
    def setUp(self):
        cells = np.zeros((160,160),dtype=np.int8)
        cells[74:87,74:87] = 100
        self.grid = r32.Grid(cells,.05,(-4.025,-4.025,0))
        self.views = TwoViews()
        edge = np.arange(-.30,.31,.05)
        self.west = np.column_stack((np.full_like(edge,-.3),edge))
        self.south = self.west[:,::-1]
        self.first = (-1.3,0.,0.)
        self.second = (0.,-1.3,math.pi/2)
        self.identity = (0,0,0)

    def observe(self, pose, points, stamp):
        self.views.observe(self.grid,pose,points,self.identity,stamp,1.1)

    def choice(self, t):
        return self.views.choose(self.grid,self.grid,self.second,self.identity,t,{},lambda goal: False)

    def test_two_sides_then_only_unseen(self):
        self.observe(self.first,self.west,1)
        self.observe(self.first,self.west,2)
        self.assertIsNone(self.choice(2))
        self.observe(self.second,self.south,3)
        self.observe(self.second,self.south,4)
        track, = self.views.tracks
        self.assertTrue(track.confirmed)
        self.assertEqual(len(track.seen),2)
        goal = self.choice(5)
        self.assertIsNotNone(goal)
        self.assertFalse(any(similar(a,goal['face_angle_odom'],35) for a in track.seen))
        self.views.dispatched(goal,5)
        self.assertEqual(len(track.seen),2)
        self.assertIsNone(self.choice(10))

    def test_single_thin_fragment_and_repeated_heading(self):
        self.grid.cells[:]=0
        self.grid.cells[74:87,74]=100
        self.observe(self.first,self.west,1)
        self.observe((-.9,-.7,.8),self.west,3)
        self.assertIsNone(self.choice(4))

    def test_all_four_views_end_box_detours(self):
        views = [(self.first,self.west),(self.second,self.south),
                 ((1.3,0.,math.pi),-self.west),((0.,1.3,-math.pi/2),-self.south)]
        for i,(pose,hits) in enumerate(views):
            self.observe(pose,hits,2*i+1)
            self.observe(pose,hits,2*i+2)
        track, = self.views.tracks
        self.assertTrue(track.confirmed)
        self.assertEqual(len(track.seen),4)
        self.assertIsNone(self.choice(12))

    def test_sideways_camera_does_not_mark_faces_seen(self):
        self.observe((-1.3,0.,math.pi),self.west,1)
        self.observe((-1.3,0.,math.pi),self.west,2)
        self.observe((0.,-1.3,-math.pi/2),self.south,3)
        self.observe((0.,-1.3,-math.pi/2),self.south,4)
        track, = self.views.tracks
        self.assertTrue(track.confirmed)
        self.assertEqual(track.seen,[])
        self.assertIsNotNone(self.choice(6))

    def test_adjacent_surfaces_single_view(self):
        points = np.concatenate((self.west,self.south))
        for t in (1,2,3):
            self.observe((-.8,-.8,.78),points,t)
        self.assertIsNone(self.choice(4))


if __name__ == '__main__':
    unittest.main()
