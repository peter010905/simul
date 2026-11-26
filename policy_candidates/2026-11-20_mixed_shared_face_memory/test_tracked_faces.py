import math
import tempfile
import unittest
from types import SimpleNamespace
import numpy as np

from osp_mission.tracked_faces import TrackedFaces
from osp_mission.center_wall import CenterWall
from osp_mission._r32_navigation_core import Grid


def rectangle(center):
    normals = (np.array([1.,0.]),np.array([-1.,0.]),np.array([0.,1.]),np.array([0.,-1.]))
    return [(center+.30*n,n) for n in normals]


class SharedCoverage(unittest.TestCase):
    def setUp(self):
        self.center = np.array([3.05,3.05])
        self.faces = rectangle(self.center)
        self.transform = (0.,0.,0.)
        self.memory = TrackedFaces()

    def test_seen_side_alias_and_unseen_adjacent(self):
        self.memory.update([(self.center,self.faces,[0.])],self.transform,0.)
        self.assertTrue(self.memory.match(self.center+[.30,.13],math.pi/4,.05).observed)
        self.assertFalse(self.memory.match(self.center+[.30,.30],math.pi/4,.05).observed)
        self.assertIsNone(self.memory.match(self.center+[.53,.11],0.,.05))
        self.memory.update([],self.transform,44.)
        self.assertTrue(self.memory.match(self.center+[.30,0.],0.,.05).observed)
        self.memory.update([],self.transform,46.)
        self.assertIsNone(self.memory.match(self.center+[.30,0.],0.,.05))

    def test_face_sweep_reaches_the_arrival_handler(self):
        from osp_mission.face_directed_look import FaceDirectedLook
        from geometry_msgs.msg import TransformStamped
        cells=np.zeros((125,125),np.int8)
        cells[55:67,55:67]=100
        grid=Grid(cells,.05,(0.,0.,0.))
        goal=CenterWall().decide(grid,grid,(2.,3.05,0.),self.transform,
                                 np.zeros((4,*cells.shape),bool),10.,600.,lambda _:False,
                                 fitted=[(self.center,self.faces,[math.pi,math.pi/2,-math.pi/2])])
        with tempfile.TemporaryDirectory() as tmp:
            tf=TransformStamped()
            tf.transform.rotation.w=1.
            owner=SimpleNamespace(p=lambda _:tmp,now=lambda:10.,inspection_camera_frame='camera',
                                  tf=SimpleNamespace(lookup_transform=lambda *args:tf),
                                  get_logger=lambda:SimpleNamespace(info=lambda _:None))
            look=FaceDirectedLook(owner)
            look.select(dict(goal,scan_angle=0.),self.transform)
            self.assertEqual(look.consume(),0.)
            self.assertEqual(look.unavailable,1)
            look.select(goal,self.transform)
            self.assertAlmostEqual(look.pending['end_odom_yaw'],goal['yaw']+math.radians(24))
            self.assertTrue(math.isfinite(look.consume()))
            self.assertEqual((look.executed,look.unavailable),(1,1))

    def test_mixed_bin_preserves_neighbour_and_short_speckle_rule(self):
        # Same grid-normal and bin, but a distinct observed wall fragment extends beyond the box.
        self.memory.update([(self.center,self.faces,[0.])],self.transform,0.)
        grid=Grid(np.zeros((130,130),np.int8),.05,(0.,0.,0.))
        dy,dx=0,1
        # The face is x=3.35 and spans y=2.75..3.35. Keep a long enough piece beyond it.
        cells=[(r,66) for r in range(56,75)]
        sample=self.memory.representative(grid,cells,dy,dx,self.transform,0.)
        self.assertIsNotNone(sample)
        self.assertGreater(sample[0][1],3.40)
        self.assertIsNone(self.memory.representative(grid,cells[:14],dy,dx,self.transform,0.))

    def test_independent_objects_and_rotated_odom(self):
        other=self.center+[1.6,0.]
        boxes=[(self.center,self.faces,[]),(other,rectangle(other),[math.pi/2])]
        self.memory.update(boxes,(1.,2.,math.pi/2),10.)
        self.memory.update(boxes,(1.,2.,math.pi/2),11.)
        self.assertEqual(len(self.memory.boxes),2)
        point=np.array([1-other[1],2+other[0]+.30])
        self.assertTrue(self.memory.match(point,math.pi/2,.05).observed)
        self.assertFalse(self.memory.match(point-[0.,1.6],math.pi/2,.05).observed)

    def test_scheduler_does_not_reopen_camera_observed_box_in_generic_branch(self):
        grid=np.full((128,128),100,np.int8)
        grid[2:-2,2:-2]=0
        grid[55:67,55:67]=100
        g=Grid(grid,.05,(0.,0.,0.))
        coverage=np.zeros((4,*grid.shape),bool)  # Generic coverage is deliberately empty.
        pose=(2.,3.05,0.)
        angles=(0.,math.pi,math.pi/2,-math.pi/2)
        scheduler=CenterWall()
        def decide(t,seen):
            return scheduler.decide(g,g,pose,self.transform,coverage,t,600.,lambda _:False,
                                    fitted=[(self.center,self.faces,seen)])
        self.assertIsNone(decide(5.,angles))
        self.assertEqual(scheduler.counts['interior'],0)
        goal=decide(10.,angles[1:])
        self.assertEqual(goal['fitted_face'],1)
        self.assertGreater(goal['scan_angle'],0.)
        self.assertGreaterEqual(goal['clearance_m'],.32)
        self.assertFalse(scheduler.tracked_faces.match(self.center+[.30,0.],0.,.05).observed)
        scheduler.dispatched(goal,10.)
        # Goal selection alone does not grant camera credit.
        self.assertFalse(scheduler.tracked_faces.match(self.center+[.30,0.],0.,.05).observed)


if __name__=='__main__':
    unittest.main()
