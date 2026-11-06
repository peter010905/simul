"""The two physical sides of a thin partition need separate camera evidence."""

import math

import numpy as np

from osp_mission.edge_policy import EdgeCoverage,boundary_faces,options,visible


def test_thin_wall_has_two_outward_normals_and_independent_inspection():
    grid=np.zeros((40,60),dtype=np.int8);grid[3:37,30]=100
    faces=boundary_faces(grid,.1)
    left=(faces['side']==3)&(faces['y']==20)&(faces['x']==30)
    right=(faces['side']==1)&(faces['y']==20)&(faces['x']==30)
    assert faces['nx'][left]<-.9 and faces['nx'][right]>.9
    memory=EdgeCoverage()
    for stamp in [0.,.01,.2]:
        memory.observe(grid,.1,(0,0),(1.05,2.05,0.),[[3.05,2.05]],stamp,665.,1.25)
    assert not memory.viewed.any()
    memory.observe(grid,.1,(0,0),(1.05,2.05,0.),[[3.05,2.05]],.4,665.,1.25)
    assert memory.viewed[3,20,30] and not memory.viewed[1,20,30]
    for stamp in [.6,.8,1.]:
        memory.observe(grid,.1,(0,0),(5.05,2.05,math.pi),[[3.05,2.05]],stamp,665.,1.25)
    assert memory.viewed[1,20,30]


def test_camera_cannot_inspect_an_occluded_face_or_an_unseen_lidar_cell():
    grid=np.zeros((40,60),dtype=np.int8);grid[3:37,20]=100;grid[3:37,30]=100
    memory=EdgeCoverage()
    for stamp in [0.,.2,.4]:
        memory.observe(grid,.1,(0,0),(1.05,2.05,0.),[[3.05,2.05]],stamp,665.,1.25)
    assert not memory.viewed.any()
    assert not visible(grid,20.5,10.5,np.array([20.5]),np.array([30.])).item()
    assert visible(grid,20.5,10.5,np.array([20.5]),np.array([20.])).item()


def test_remaining_wall_side_generates_only_reachable_free_goals():
    grid=np.zeros((45,65),dtype=np.int8)
    grid[0,:]=grid[-1,:]=grid[:,0]=grid[:,-1]=100;grid[5:40,30]=100
    viewed=np.ones((4,)+grid.shape,dtype=bool);viewed[1,5:40,30]=False
    goals,_=options(grid,.1,(0,0),(4.,2.),viewed=viewed)
    assert goals and max(g['features'][0] for g in goals)>0
    assert all(grid[g['cell']]==0 for g in goals)
    viewed[:]=True
    assert not options(grid,.1,(0,0),(4.,2.),viewed=viewed)[0]
