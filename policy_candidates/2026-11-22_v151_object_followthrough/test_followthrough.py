"""Bounded object follow-through on geometry independent of archived layouts."""
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from osp_mission.followthrough import FollowThrough, isolated_fit


class Grid:
    resolution=.05
    def __init__(self):
        self.cells=np.zeros((100,100),int)
        self.cells[44:57,44:57]=100
    def index(self,x,y):
        return round(y/.05)+50,round(x/.05)+50
    def line_clear(self,*args):
        return True


def square(x=0.):
    center=np.array([x,0.]); normals=np.array([[1.,0.],[0.,1.],[-1.,0.],[0.,-1.]])
    return center,[(center+.3*n,n) for n in normals]


def option(i,seconds=8.,**flags):
    center,faces=square()
    face,n=faces[i]
    base={'normal_odom':math.atan2(n[1],n[0]),'surface_map':face.tolist(),
          'x':float(n[0]),'y':float(n[1]),'previous_attempts':0}
    return SimpleNamespace(goal=dict(base,**flags),seconds=seconds)


class FollowTests(unittest.TestCase):
    def setUp(self):
        self.grid=Grid()
        self.policy=FollowThrough()
        self.seed={'r32_kind':'interior_surface','surface_odom':[-.3,0.],'normal_odom':math.pi,
                   'survey_nav_odom':[-1,0.],'survey_expected_s':54.}
        self.policy.dispatched(self.seed,10.)
        self.start=patch('osp_mission.followthrough.local_boxes',return_value=[square()])
        self.start.start()
        self.addCleanup(self.start.stop)

    def choose(self,t=49.,p=(-1.,0.,0.),options=None,waiting=None,remaining=100.):
        if options is None:options=[option(1)]
        if waiting is None:waiting={}
        eligible={'interior':options,**{key:[object()] for key in waiting}}
        return self.policy.choose(self.grid,p,(0.,0.,0.),eligible,waiting,90.,t,remaining)

    def test_local_clock_starts_after_actual_arrival(self):
        self.assertIsNone(self.choose(p=(2.0,0.,0.)))
        opt,extra=self.choose()  # 39s after dispatch, already at its destination.
        self.assertEqual(extra['survey_follow_step'],1)
        self.assertEqual(extra['survey_follow_budget_s'],30.)
        self.assertIsNone(self.choose(t=80.))

    def test_only_distinct_unseen_feasible_options_under_other_deadlines(self):
        for name in ('previous_attempts','completed_box','tagged_box','reopened_face'):
            self.assertIsNone(self.choose(options=[option(1,**{name:1})]))
        self.assertIsNone(self.choose(options=[option(2)]))  # originally visited face
        self.assertIsNone(self.choose(remaining=8.))  # 2s mission reserve
        self.assertIsNone(self.choose(waiting={'wall':-40.}))
        self.assertIsNone(self.choose(waiting={'frontier':-40.}))
        self.assertIsNone(self.choose(options=[]))

    def test_observed_options_do_not_survive_beyond_two_distinct_goals(self):
        _,first=self.choose()
        self.policy.dispatched(first,49.)
        self.assertIsNone(self.choose(t=55.,p=(0.,1.,0.),options=[option(1)]))
        _,second=self.choose(t=55.,p=(0.,1.,0.),options=[option(3)])
        self.assertEqual(second['survey_follow_step'],2)
        self.policy.dispatched(second,55.)
        self.assertIsNone(self.choose(t=61.,p=(0.,-1.,0.),options=[option(0)]))
        self.policy.dispatched(self.seed,64.)
        self.assertIsNone(self.choose(t=67.))
        self.policy.dispatched({'r32_kind':'wall_surface','weak_wall':True},68.)
        self.assertIsNone(self.choose(t=70.))

    def test_shared_components_multiple_hosts_and_bad_sightline_reject(self):
        with patch('osp_mission.followthrough.local_boxes',return_value=[square(),square(.05)]):
            self.assertIsNone(self.choose())
        self.grid.cells[50,:55]=100  # a wall through the object leaves the fitted neighborhood
        self.assertIsNone(self.choose())
        self.grid=Grid();self.grid.line_clear=lambda *args:False
        self.assertIsNone(self.choose())

    def test_host_drift_does_not_transfer_local_session(self):
        _,first=self.choose()
        self.policy.dispatched(first,49.)
        self.policy.window=(49.,self.policy.window[1],(1.,1.))
        self.assertIsNone(self.choose(t=52.,p=(0.,1.,0.),options=[option(0)]))

    def test_close_to_seed_pose_yet_far_from_object_ends_local_visit(self):
        self.seed['survey_nav_odom']=[4.,0.]
        self.policy.dispatched(self.seed,10.)
        self.assertIsNone(self.choose(p=(4.,0.,0.)))


if __name__=='__main__':
    unittest.main()
