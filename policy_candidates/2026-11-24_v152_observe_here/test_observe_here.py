import math
import numpy as np
import pytest
from osp_mission._r32_navigation_core import Grid
from osp_mission.center_wall import CenterWall


def setup():
    schedule = CenterWall()
    schedule.survey.camera = (443.53,1.25)
    schedule.wall_camera = (0.05,0.,0.)
    grid = Grid(np.zeros((80,80),dtype=np.int8),.05,(-2.,-2.,0.))
    allowed = grid.cells == 0
    goal = dict(x=.45,y=0.,yaw=0.,distance=.45,stationary=False,r32_kind='wall_surface',
                joint_views=1,joint_walk_m=.45,surface_map=[1.18,0.],normal_odom=math.pi,
                scan_angle=math.radians(36))
    def choose():
        return schedule.observe_wall_here(goal,grid,grid,allowed,allowed,(0.,0.,0.),(0.,0.,0.),
                                          1.676,lambda g:False)
    return schedule,grid,goal,choose


def test_short_visible_wall_needs_no_nav_goal_and_keeps_camera_sweep():
    _,_,goal,choose=setup()
    look=choose()
    assert look['stationary'] and look['original_wall_route_m']==goal['joint_walk_m']
    assert look['x']==look['y']==0. and look['scan_angle']==goal['scan_angle']
    assert not goal['stationary']


@pytest.mark.parametrize('case',['multi','box','occluded','far','back','repeat','stale','narrow_fov','turn_limit','unsafe'])
def test_no_substitution_without_equivalent_safe_observation(case):
    s,g,goal,choose=setup()
    if case=='multi': goal['joint_views']=2
    if case=='box': goal['r32_kind']='interior_surface'
    if case=='occluded': g.cells[:,g.index(.55,0.)[1]]=100
    if case=='far': goal['surface_map']=[1.85,0.]
    if case=='back': goal['normal_odom']=0.
    if case=='repeat': s.last_wall_look_pose=(0.,0.)
    if case=='stale': s.wall_camera=None
    if case=='narrow_fov': s.survey.camera=(443.53,.4)
    if case=='turn_limit': s.wall_camera=(.05,0.,math.pi)
    if case=='unsafe':
        # Change the actual safe mask used by the closure as well as the input grid.
        allowed=(g.cells==0); allowed[g.index(0.,0.)]=False
        assert s.observe_wall_here(goal,g,g,allowed,allowed,(0.,0.,0.),(0.,0.,0.),1.676,lambda v:False) is goal
        return
    assert choose() is goal
