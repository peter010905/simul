"""An opportunistic glance cannot use hidden or already inspected surfaces."""

import math
import numpy as np

from osp_mission.edge_policy import boundary_faces
from osp_mission.travel_glance import select


def test_glance_finds_side_surface_but_ignores_front_and_confirmed_faces():
    grid=np.zeros((60,60),dtype=np.int8);grid[40,5:55]=100
    faces=boundary_faces(grid,.1);viewed=np.zeros((4,)+grid.shape,dtype=bool)
    camera=(3.,2.,0.)
    goal=select(grid,.1,(0.,0.),camera,faces,viewed,665.,1.25)
    assert goal is not None and abs(goal['yaw']-math.pi/2)<.4
    assert select(grid,.1,(0.,0.),(3.,2.,math.pi/2),faces,viewed,665.,1.25) is None
    viewed[:]=True
    assert select(grid,.1,(0.,0.),camera,faces,viewed,665.,1.25) is None


def test_glance_does_not_look_through_an_occluder_or_toward_a_back_face():
    grid=np.zeros((60,60),dtype=np.int8);grid[40,5:55]=100;grid[30,:]=100
    faces=boundary_faces(grid,.1);viewed=np.ones((4,)+grid.shape,dtype=bool)
    viewed[0,40,5:55]=False
    assert select(grid,.1,(0.,0.),(3.,2.,0.),faces,viewed,665.,1.25) is None
    grid[30,:]=0;faces=boundary_faces(grid,.1)
    viewed[:]=True;viewed[2,40,5:55]=False
    assert select(grid,.1,(0.,0.),(3.,2.,0.),faces,viewed,665.,1.25) is None
