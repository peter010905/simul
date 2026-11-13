"""Front-facing search candidates must survive narrow-passage bucketing."""

import numpy as np
from scipy.ndimage import binary_dilation

from osp_mission.directional_policy import options as original_options
from osp_mission.face_policy import options, surface_weights


def test_surface_viewpoints_include_both_sides_of_thin_partition():
    grid=np.zeros((45,65),dtype=np.int8)
    grid[0,:]=grid[-1,:]=grid[:,0]=grid[:,-1]=100
    grid[8:36,30]=100
    viewed=np.ones((8,)+grid.shape,dtype=bool)
    viewed[:,8:36,30]=False
    goals,_=options(grid,.1,(0,0),(1.5,2.),viewed=viewed,balanced=False)
    assert any(1.8<g['x']<3 and g['features'][0]>0 for g in goals)
    assert any(3<g['x']<4.4 and g['features'][0]>0 for g in goals)
    assert all(grid[g['cell']]==0 for g in goals)
    # An enclosing wall makes the opposite half inaccessible.
    grid[1:-1,30]=100
    goals,_=options(grid,.1,(0,0),(1.5,2.),viewed=viewed,balanced=False)
    assert all(g['x']<3 for g in goals)


def test_new_front_views_survive_open_point_bucket_selection():
    grid=np.zeros((40,60),dtype=np.int8)
    grid[0,:]=grid[-1,:]=grid[:,0]=grid[:,-1]=100
    grid[18:22,47:50]=100
    old,_=original_options(grid,.1,(0,0),(2.,2.))
    new,_=options(grid,.1,(0,0),(2.,2.),balanced=False)
    old_cells={g['cell'] for g in old}
    extra=[g for g in new if g['cell'] not in old_cells]
    assert any(g['x']>5.1 and 1.6<g['y']<2.4 and g['features'][0]>0 for g in extra)


def test_object_balance_retains_walls_but_values_small_obstacle_faces():
    occupied=np.zeros((40,80),dtype=bool)
    occupied[1,1:78]=True
    occupied[20:23,20:23]=True
    surface=occupied&binary_dilation(~occupied)
    weights=surface_weights(occupied,surface,.1)
    assert .25<=weights[1,20]<weights[20,20]<=1


def test_still_gaze_and_shorter_sweep_keep_their_real_observation_costs():
    import math
    from osp_mission.face_policy import make_scans,choose_timed
    actions=make_scans(np.array([-.1,0.,.1]),np.ones(3),0.)
    still=actions[0]
    assert still[0]==3 and still[2]==0 and abs(still[1])<.2
    candidates=[dict(scan_angle=0.,features=np.array([3.,0.,.5,0.,0.])),
                dict(scan_angle=2*math.pi,features=np.array([3.,0.,.5,0.,0.]))]
    assert choose_timed(candidates) is candidates[0]
    # A clockwise sweep is allowed when it avoids an unnecessary alignment.
    actions=make_scans(np.array([-.9,-.6,-.3]),np.ones(3),0.)
    assert any(a[2]<0 for a in actions)


def test_endpoint_face_candidates_use_single_surface_observation_memory():
    grid=np.zeros((40,60),dtype=np.int8)
    grid[0,:]=grid[-1,:]=grid[:,0]=grid[:,-1]=100
    grid[18:22,47:50]=100
    viewed=np.zeros_like(grid,dtype=bool)
    goals,_=options(grid,.1,(0,0),(2.,2.),viewed=viewed,balanced=False,directional=False)
    assert any(g['x']>5.1 and 1.6<g['y']<2.4 and g['features'][0]>0 for g in goals)
    viewed[:]=True
    goals,_=options(grid,.1,(0,0),(2.,2.),viewed=viewed,balanced=False,directional=False)
    assert not goals


def test_search_prior_does_not_reward_a_large_obstacle_more_than_a_small_one():
    from osp_mission.face_policy import prior_weights
    occupied=np.zeros((80,100),dtype=bool)
    occupied[1,1:95]=True
    occupied[30:34,15:19]=True
    occupied[40:46,50:70]=True
    surface=occupied&binary_dilation(~occupied)
    weights=prior_weights(occupied,surface,.1)
    wall=weights[1,:][surface[1,:]].sum()*.1
    small=(weights[30:34,15:19]*surface[30:34,15:19]).sum()*.1
    large=(weights[40:46,50:70]*surface[40:46,50:70]).sum()*.1
    assert abs(small-large)<1e-9 and abs(wall+small+large-100)<1e-9
