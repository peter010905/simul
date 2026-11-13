import math

import numpy as np

from osp_mission.edge_policy import options


def test_surrounded_viewpoint_rewards_only_its_bounded_visible_sector():
    grid=np.zeros((81,81),dtype=np.int8)
    grid[10:71,10]=grid[10:71,70]=100
    grid[10,10:71]=grid[70,10:71]=100
    goals,_=options(grid,.1,(0,0),(4.,3.),collect_faces=True,
        point_sampler=lambda *args:[(40,40)])
    assert len(goals)==1
    goal=goals[0]
    assert 0<goal['scan_angle']<=2*math.pi/3
    faces=goal['visible_faces']
    bearings=np.arctan2((faces[:,1]+.5)-40.5,(faces[:,2]+.5)-40.5)
    center=goal['yaw']+math.pi/3
    relative=(bearings-center+math.pi)%(2*math.pi)-math.pi
    assert np.all(np.abs(relative)<math.radians(55)+1e-8)
    assert math.isclose(goal['features'][0],len(faces)*.1)


def test_default_goals_never_request_a_full_circle_after_bounded_sweep_change():
    grid=np.zeros((65,65),dtype=np.int8);grid[0,:]=grid[-1,:]=grid[:,0]=grid[:,-1]=100
    grid[25:40,25:40]=100
    goals,_=options(grid,.1,(0,0),(1.,1.))
    assert goals and all(g['scan_angle']<=2*math.pi/3+1e-9 for g in goals)
