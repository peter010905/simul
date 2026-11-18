import math
import numpy as np
from osp_mission.robust_scenarios import make_robust
from osp_mission.scenarios import valid
from osp_mission.surface_policy import normals,options,choose
from osp_mission.fast_sim import SearchEnv


def test_dense_hidden_targets_are_distinct_accessible_obstacle_faces():
    cfg=make_robust(8000001,12,'hidden_obstacles')
    assert cfg==make_robust(8000001,12,'hidden_obstacles')
    assert valid(cfg) and len(cfg['obstacles'])==12
    assert len({t['carrier_obstacle'] for t in cfg['targets']})==3
    for t in cfg['targets']:
        assert t['mount']=='obstacle'
        dot=(cfg['start'][0]-t['x'])*math.cos(t['yaw'])+(cfg['start'][1]-t['y'])*math.sin(t['yaw'])
        assert dot<0


def test_surface_normals_face_free_space_and_goals_are_reachable():
    grid=np.full((40,60),-1,dtype=np.int8);grid[5:35,5:50]=0;grid[5:35,30]=100
    ny,nx=normals(grid,.1)
    assert nx[20,29]<-.9
    candidates,_=options(grid,.1,(0,0),(1.2,1.2))
    assert candidates and all(c['x']<3 for c in candidates)
    goal=choose(candidates)
    assert 2*math.pi/3<=goal['scan_angle']<=2*math.pi


def test_detection_after_deadline_does_not_complete_mission():
    cfg=dict(seed=1,start=[2.,2.,0.],obstacles=[],targets=[dict(id=7,x=3.,y=2.,yaw=math.pi,size=.3)])
    e=SearchEnv(cfg,seed=1,camera_width=960,detection_dropout=0.)
    e.confirmations[7]=2;e.last_confirmation[7]=0.;e.elapsed=601.;e.deadline=600.
    e.detect()
    assert 7 not in e.found and e.confirmations[7]==2


def test_close_surface_visibility_uses_endpoint_cell_margin():
    from osp_mission.surface_v2_policy import options as improved_options
    grid=np.full((45,45),-1,dtype=np.int8)
    grid[2:42,10:24]=0;grid[2:42,9]=100;grid[2:42,24]=100
    old,_=options(grid,.1,(0,0),(1.7,1.7))
    new,_=improved_options(grid,.1,(0,0),(1.7,1.7))
    assert old and new
    assert max(c['features'][0] for c in old)==0
    assert max(c['features'][0] for c in new)>0
    assert all(1.<c['x']<2.4 for c in new)


def test_fixed_camera_clock_does_not_invent_extra_frames():
    cfg=dict(seed=1,start=[2.,2.,0.],obstacles=[],targets=[dict(id=7,x=3.,y=2.,yaw=math.pi,size=.3)])
    e=SearchEnv(cfg,seed=1,camera_width=960,detection_dropout=0.,camera_hz=5)
    assert e.confirmations[7]==1
    for time in [.01,.05,.10,.19]:
        e.elapsed=time;e.scan()
    assert e.confirmations[7]==1
    e.elapsed=.2;e.scan();assert e.confirmations[7]==2
    e.elapsed=.4;e.scan();assert 7 in e.found


def test_fixed_rate_rotation_stops_at_deadline_and_observes_during_hold():
    cfg=dict(seed=2,start=[2.,2.,0.],obstacles=[],targets=[])
    e=SearchEnv(cfg,seed=2,camera_width=960,camera_hz=5)
    e.rotate(math.pi,deadline=.45)
    assert abs(e.elapsed-.45)<1e-9
    assert len(e.trace)>=3
    before=len(e.trace);e.hold(.5,deadline=1.)
    assert abs(e.elapsed-.95)<1e-9 and len(e.trace)>before


def test_thin_partition_has_a_normal_axis_and_two_independent_faces():
    from osp_mission.directional_policy import normals as axes, options as directional_options
    grid=np.zeros((45,65),dtype=np.int8)
    grid[0,:]=grid[-1,:]=grid[:,0]=grid[:,-1]=100
    grid[5:40,30]=100
    ny,nx=axes(grid,.1)
    assert abs(nx[20,30])>.9 and abs(ny[20,30])<.1
    viewed=np.ones((8,)+grid.shape,dtype=bool)
    viewed[:,5:40,30]=False
    # Left-facing views cannot complete the right-hand face of the same cell.
    viewed[[7,0,1],5:40,30]=True
    goals,_=directional_options(grid,.1,(0,0),(4.,2.),viewed=viewed)
    assert goals and max(g['features'][0] for g in goals)>0
    viewed[:,5:40,30]=True
    goals,_=directional_options(grid,.1,(0,0),(4.,2.),viewed=viewed)
    assert not goals


def test_surrogate_respects_detector_optical_depth_limit():
    cfg=dict(seed=1,start=[1.,2.,0.],obstacles=[],targets=[dict(id=7,x=10.,y=2.,yaw=math.pi,size=.3)])
    e=SearchEnv(cfg,seed=1,camera_width=960,detection_dropout=0.)
    assert e.confirmations.get(7,0)==0  # Optical depth is 9 m.
    e.elapsed=.2;e.yaw=math.pi/6;e.detect()
    assert e.confirmations.get(7,0)==1  # Optical z < 8 m, still within the image.


def test_shield_stops_before_entering_collision_cell_at_grid_boundary():
    from osp_mission.fast_sim import motion_blocked
    grid=np.zeros((20,30),dtype=np.int8);grid[11,:]=100
    # Regression from seed 8200066: the continuous destination is just over
    # 0.30 m from the wall centre, but its cell is only one 0.20 m cell away.
    assert motion_blocked(grid,.2,4.1000000000000005,2.0)
    assert not motion_blocked(grid,.2,4.1,1.7)
    # The shield receives no ground truth and cannot invent an unknown wall.
    grid[11,:]=-1
    assert not motion_blocked(grid,.2,4.1,2.0)
    # The extra discrete check uses the evaluator's body radius, so it does
    # not inflate a diagonal cell into a fictitious collision.
    grid[:]=0;grid[11,11]=100
    assert not motion_blocked(grid,.2,2.,2.)
