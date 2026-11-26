import math
from types import SimpleNamespace
import numpy as np
from osp_mission.rescue360 import Rescue360,rotation_seconds
from osp_mission._r32_navigation_core import Grid


def owner(tmp_path):
    clock=[0.]
    params=dict(rescue_after_s=360.,rescue_travel_speed=.20,rescue_spin_rate=.5,
                rescue_spin_accel=1.,rescue_settle_s=1.5,duration=600.,output_dir=str(tmp_path))
    events=[]
    node=SimpleNamespace(p=params.__getitem__,now=lambda:clock[0],started=0.,
        camera_diagnostics=dict(coverage_updates=0,accepted=0),goal_handle=None,pending=False,
        scan_after=True,plan_after=0.,kind='nav',recovery_pending=False,
        cancel_action=lambda reason:events.append(reason),zero=lambda:events.append('zero'))
    return node,clock,events


def candidate(x=1.,gain=4.,frontier=0.,distance=2.):
    return dict(x=x,y=1.,yaw=0.,features=np.array([gain,frontier,distance,0.,0.]),distance=distance)


def select(rescue,goals,nav=None):
    g=Grid(np.zeros((80,80),np.int8),.05,(0,0,0))
    if nav is None:nav=np.ones_like(g.cells,bool)
    return rescue.choose(goals,(.5,.5,0.),(0,0,0),g,g,nav,lambda _:False,np.ones(5))


def test_before_six_minutes_preserves_current_action_then_transitions(tmp_path):
    node,clock,events=owner(tmp_path);r=Rescue360(node);node.goal_handle=True
    clock[0]=359.999
    assert not r.transition() and not events and not r.active
    clock[0]=360.
    assert r.transition() and r.active and events==['rescue_phase_due','zero']


def test_registered_marker_approach_delays_phase_change(tmp_path):
    node,clock,events=owner(tmp_path);r=Rescue360(node)
    node.batch_approach_id=7
    clock[0]=370
    assert not r.transition() and not r.active and not events
    node.batch_approach_id=None
    assert not r.transition() and r.active and not node.scan_after


def test_camera_surface_can_win_with_zero_frontier_gain(tmp_path):
    node,clock,_=owner(tmp_path);clock[0]=370;r=Rescue360(node)
    best=select(r,[candidate(1.,8.,0.),candidate(2.,1.,2.)])
    assert best['x']==1. and best['rescue_surface_gain']==8. and best['scan_angle']==2*math.pi


def test_budget_includes_translation_acceleration_rotation_and_dwell(tmp_path):
    node,clock,_=owner(tmp_path);r=Rescue360(node);clock[0]=570.
    assert abs(r.rotation_s-(2*math.pi/.5+.5))<1e-9
    best=select(r,[candidate(1.,10.,0.,20.),candidate(2.,2.,0.,1.)])
    assert best['x']==2. and best['rescue_expected_s']>rotation_seconds(2*math.pi,.5,1.)+1.5+5.
    clock[0]=590
    assert select(r,[candidate()]) is None


def test_disconnected_goals_and_completed_sites_are_excluded(tmp_path):
    node,clock,_=owner(tmp_path);r=Rescue360(node);clock[0]=370
    nav=np.ones((80,80),bool);nav[20,20]=False
    assert select(r,[candidate()],nav) is None
    r.visited=[np.array([1.,1.])]
    assert select(r,[candidate()]) is None


def test_successful_action_requires_full_rotation_and_valid_camera_exposures(tmp_path):
    node,clock,_=owner(tmp_path);r=Rescue360(node);clock[0]=370
    goal=select(r,[candidate()])
    r.dispatched(goal);r.outcome('nav',4,'',0.)
    assert r.spin_pending and not r.visited
    r.spin_sent();r.outcome('spin',4,'',2*math.pi)
    assert not r.visited and r.completed_sweeps==0
    clock[0]=420
    goal=select(r,[candidate()]);r.dispatched(goal);r.outcome('nav',4,'',0.)
    r.spin_sent();node.camera_diagnostics['coverage_updates']=20
    r.outcome('spin',4,'',2*math.pi)
    assert len(r.visited)==1 and r.completed_sweeps==1
    clock[0]+=1.5
    assert r.waiting()
    node.camera_diagnostics['coverage_updates']+=3
    assert not r.waiting()


def test_partial_turns_and_sensor_pauses_do_not_fabricate_a_full_sweep(tmp_path):
    node,clock,_=owner(tmp_path);r=Rescue360(node);clock[0]=370
    r.dispatched(select(r,[candidate()]));r.outcome('nav',4,'',0.)
    r.spin_sent();node.camera_diagnostics['coverage_updates']=10
    r.outcome('spin',5,'processed_camera_stale',3.)
    assert r.spin_running and r.target is not None and not r.visited
    node.camera_diagnostics['coverage_updates']+=10
    r.outcome('spin',4,'',3.25)
    assert len(r.visited)==1


def test_actual_mission_turn_honors_full_sweep_even_with_default_bounded_scan(tmp_path):
    from osp_mission.mission import Mission
    stub=SimpleNamespace(rescue=SimpleNamespace(spin_pending=True),next_scan_angle=2*math.pi/3)
    assert Mission.observation_turn(stub)==2*math.pi


def test_full_scan_sampler_counts_surfaces_outside_single_heading():
    from osp_mission.edge_policy import options
    cells=np.zeros((37,37),np.int8);cells[[0,-1],:]=100;cells[:,[0,-1]]=100
    args=(cells,.05,(0,0),(.9,.9))
    partial,_=options(*args,inspection_focal=333.)
    full,_=options(*args,inspection_focal=333.,full_scans=True)
    assert full and partial
    assert max(g['features'][0] for g in full)>max(g['features'][0] for g in partial)
    assert all(g['scan_angle']==2*math.pi for g in full)
