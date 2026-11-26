from concurrent.futures import Future
from types import SimpleNamespace
import math

import numpy as np
import pytest

from osp_mission._r32_navigation_core import Grid
from osp_mission.box_continuity import BoxContinuity
from osp_mission.center_wall import CenterWall
from osp_mission.interior_priority import InteriorPriority
from osp_mission.marker_approach import MarkerApproach
from osp_mission.marker_focus import MarkerFocus
from osp_mission.navigation_flow import NavigationFlow
from osp_mission.work_rules import FaceWork, marker_task_dispatched


def face(**extra):
    return dict(dict(x=4.,y=3.,yaw=0.,scan_angle=.5,distance=1.,features=np.zeros(5),
        r32_kind='interior_surface',face_inspection=True,surface_map=[3.,3.],
        surface_odom=[3.,3.],normal_odom=0.,batch_box=[3.,3.5],
        previous_attempts=0,completed_box=False),**extra)


def room():
    data=np.zeros((90,110),np.int8)
    data[[0,-1],:]=100
    data[:,[0,-1]]=100
    return Grid(data,.1,(0.,0.,0.))


def hint():
    f=MarkerFocus()
    assert f.observe(dict(provisional=True,id=42,stamp=10.,map_point=[3.,3.,.2],
        observation_frame='map',min_edge_pixels=8.),[1.,3.],10.1)
    return f


def test_only_the_final_marker_task_consumes_a_retry():
    f=hint(); a=MarkerApproach()
    for _ in range(8):
        proposed=f.choose(np.zeros((80,80),np.int8),.1,(0.,0.),(1.,3.),0.,[],{},10.15)
        assert proposed['focus_id']==42
    assert not f.attempts and not f.last_attempt
    marker_task_dispatched(f,a,dict(approach_id=7),10.2)
    assert not f.attempts and a.last_attempt=={7:10.2}
    marker_task_dispatched(f,a,proposed,10.3)
    assert f.attempts=={42:1}
    assert not f.request_interrupt(42,{},10.6,deferred=True)


def test_focus_followup_is_not_swallowed_when_a_proposal_was_only_evaluated():
    f=hint()
    assert f.choose(np.zeros((80,80),np.int8),.1,(0.,0.),(1.,3.),0.,[],{},10.15)
    assert f.request_interrupt(42,{},10.6,deferred=True)


def test_routed_approach_is_committed_once():
    f=MarkerFocus();a=MarkerApproach()
    marker_task_dispatched(f,a,dict(approach_id=7,routed_offset=.3),2.)
    assert a.routed_views=={7:[.3]} and a.last_attempt[7]==2.


def test_success_and_failure_exclusion_are_distinct():
    f=NavigationFlow();g=face()
    f.selected(g,(0.,0.,0.));f.record_navigation_result(4,'',1.)
    assert not f.blocked(g,(0.,0.,0.),2.)
    f.record_navigation_result(6,'navigation_stalled',3.)
    assert f.blocked(g,(0.,0.,0.),4.)
    assert not f.blocked(g,(0.,0.,0.),64.)


def test_nav_success_does_not_grant_camera_coverage_or_failure():
    work=FaceWork();g=face();work.dispatched(g)
    work.result('nav',4,'',2.)
    work.result('spin',4,'',3.)
    row=work.row(g['surface_odom'],g['normal_odom'])
    assert row['status']=='PENDING' and not row['failures']


def test_failed_face_reopens_only_on_meaningful_change_and_is_globally_bounded():
    w=FaceWork();g=face();point=g['surface_odom']
    geometry=[0]*49;w.probe(point,0.,geometry)
    for i in range(2):
        w.dispatched(g);w.result('nav',6,'navigation_stalled',i*46.)
    assert not w.allowed(point,0.,100.)
    tiny=geometry.copy();tiny[0]=1;w.probe(point,0.,tiny)
    assert not w.allowed(point,0.,100.)
    for epoch in range(2):
        geometry=[epoch+1]*49;w.probe(point,0.,geometry)
        assert w.allowed(point,0.,200.)
        for i in range(2):
            w.dispatched(g);w.result('nav',6,'navigation_stalled',200.+epoch*100.+i*46.)
    w.probe(point,0.,[8]*49)
    assert not w.allowed(point,0.,600.)


def test_named_marker_pause_preserves_unfinished_face_without_failure():
    w=FaceWork();g=face();w.dispatched(g)
    w.result('nav',5,'registered_marker',3.)
    assert not w.attempts(g['surface_odom'],0.)


def test_box_budget_cannot_be_reopened_by_normal_priority():
    c=BoxContinuity();g=face();c.dispatched(g,0.)
    assert c.available(g,54.)
    assert not c.available(g,55.) and not c.available(g,144.)
    assert c.available(face(batch_box=[7.,7.]),56.)
    assert c.available(g,145.)
    c.dispatched(g,145.)
    assert c.entered==145. and c.available(g,146.)


def test_close_box_is_scored_against_a_new_frontier():
    grid=room();_,d=grid.reachable((2.,3.),.32)
    near=face(x=3.,marker_gain=.18)
    frontier=dict(x=8.,y=3.,yaw=0.,distance=6.,features=np.array([2.,8.,6.,0.,0.]))
    g=InteriorPriority().choose(grid,d,(0.,0.,0.),[(1.,near)],[frontier],[],
        [(2.,3.),(2.1,3.),(2.2,3.),(2.3,3.)],500.,lambda _:False)
    assert g['r32_kind']=='interior_frontier'


@pytest.mark.parametrize('mode',['resume','leave'])
def test_region_transit_branch_is_reachable_after_wall_lap(mode):
    grid=room();s=CenterWall();s.wall_lap.finish('test')
    s.region_transit.mode=lambda *args:mode
    expected=dict(x=6.,y=5.,r32_kind='interior_frontier',features=np.zeros(5))
    s.region_transit.choose=lambda *args:expected
    s.survey.choose=lambda *args,**kwargs:None
    result=s.decide(grid,grid,(2.,3.,0.),(0.,0.,0.),np.zeros((4,)+grid.cells.shape,bool),200.,600.,lambda _:False)
    assert result is expected and s.reason=='region_'+mode


def test_same_geometry_cannot_infinite_retry_failed_face():
    w=FaceWork();g=face()
    w.probe(g['surface_odom'],0.,[0]*49)
    for t in (0.,46.):
        w.dispatched(g);w.result('nav',6,'blocked',t)
    for t in (100.,300.,10000.):
        assert not w.allowed(g['surface_odom'],0.,t)


def test_legacy_start_radius_never_opens_occupied_cells():
    from osp_mission.edge_policy import options
    g=room();g.cells[27:34,17:24]=100
    assert options(g.cells,.1,(0.,0.),(2.05,3.05),start_ignore_radius=.3)[0]==[]


def test_no_final_navigation_success_adds_a_hard_exclusion():
    from osp_mission.mission import Mission
    owner=SimpleNamespace(action_generation=1,goal_handle=object(),cancel_started=None,
        state='running',kind='nav',cancel_reason='',now=lambda:2.,started=0.,
        contact_inspection=None,contact_pending=False,hybrid=None,near_pass=None,
        navigation_flow=None,exploration_memory=None,excluded=[],destination=(1.,2.))
    f=Future();f.set_result(SimpleNamespace(status=4))
    Mission.completed(owner,f)
    assert owner.excluded==[]


def test_coverage_size_prior_is_shared():
    from osp_mission.edge_policy import readable_distance
    from osp_mission.visual_survey import VisualSurvey
    s=VisualSurvey();s.camera=(400.,1.2)
    assert s.coverage_limit()==readable_distance(400.)


def test_regular_wall_destinations_stay_disabled_in_interior_stage():
    grid=room();s=CenterWall();s.wall_lap.finish('test');called=[]
    s.joint.choose=lambda *args,**kwargs:called.append(kwargs) or None
    s.survey.choose=lambda *args,**kwargs:None
    s.decide(grid,grid,(2.,3.,0.),(0.,0.,0.),np.zeros((4,)+grid.cells.shape,bool),200.,600.,lambda _:False)
    assert called and called[0]['allow_wall_destinations'] is False
