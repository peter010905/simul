from collections import deque
from types import SimpleNamespace
import math

import numpy as np
import pytest

from osp_mission.box_continuity import BoxContinuity
from osp_mission.box_batch import BoxBatch
from osp_mission.center_wall import CenterWall
from osp_mission.hybrid import Hybrid, PrintMemory
from osp_mission._r32_navigation_core import Grid, travel_distances
from osp_mission.box_tour import short_tour


def face(**kw):
    return dict(r32_kind='interior_surface',face_inspection=True,surface_odom=[3.,3.],
                normal_odom=0.,batch_box=[3.,3.5],face_dispatch_elapsed=1.,
                region_face=('box',6,7,0),region_anchor=[3.,3.],previous_attempts=0,completed_box=False,
                x=4.,y=3.,**kw)


def test_canceled_face_rolls_back_only_its_dispatch_and_resumes():
    s=CenterWall();g=face()
    s.dispatched(g,1.)
    assert len(s.history)==1 and s.joint.stops==1
    s.face_result('nav',5,'registered_marker',3.)
    assert len(s.history)==0 and s.joint.stops==0 and not s.joint.attempted
    grid=Grid(np.zeros((60,60),np.int8),.1,(0.,0.,0.))
    resumed=s.continuity.choose([(1.,g)],4.,500.,grid,np.ones((60,60),bool))
    assert resumed['box_face_resume']
    assert resumed['face_resume_count']==1
    s.dispatched(resumed,4.)
    assert not s.continuity.suspended
    s.face_result('nav',4,'',5.)
    assert s.continuity.protecting(6.)
    s.continuity.begin_sweep(.5,6.)
    s.face_result('spin',4,'',7.)
    assert not s.continuity.protecting(8.)
    assert len(s.history)==1


def test_repeated_named_interruptions_are_bounded():
    s=CenterWall();g=face()
    grid=Grid(np.zeros((60,60),np.int8),.1,(0.,0.,0.))
    for n in range(3):
        s.dispatched(g,1.+n*4)
        s.face_result('nav',5,'marker_hint',2.+n*4)
        if n<2:
            g=s.continuity.choose([(1.,face())],3.+n*4,500.,grid,np.ones((60,60),bool))
            assert g['face_resume_count']==n+1
    assert not s.continuity.suspended


def test_failure_keeps_failure_history_and_releases_white_protection():
    s=CenterWall();s.dispatched(face(),1.)
    s.face_result('nav',6,'navigation_stalled',2.)
    assert len(s.history)==1 and not s.continuity.suspended
    assert not s.continuity.protecting(3.)


def test_white_leg_is_held_and_black_border_still_interrupts():
    h=Hybrid()
    h.schedule=SimpleNamespace(continuity=BoxContinuity())
    h.schedule.continuity.dispatched(face(),1.)
    owner=SimpleNamespace(now=lambda:2.,started=0.,get_clock=lambda:SimpleNamespace(
        now=lambda:SimpleNamespace(nanoseconds=2_000_000_000)))
    h.memory.defer_white=True
    for t in (1.6,1.8,2.):
        h.memory.observe(dict(stamp=t,point=[4.,3.],observer=[2.,3.],white_leg=True),t)
    assert h.memory.tracks[0]['count']==3
    assert h.memory.tracks[0]['held_until']==92.
    assert not h.has_fresh_print(owner)
    h.memory.observe(dict(stamp=40.,point=[8.,3.],observer=[6.,3.]),40.)
    assert h.memory.tracks[0]['white_leg']  # original leg persists past ordinary 30s age
    h.memory.tracks.append(dict(count=3,stamp=2.,white_leg=False))
    assert h.has_fresh_print(owner)


def test_box_identity_survives_marker_detour():
    b=BoxBatch();b.dispatched(face())
    b.dispatched(dict(print_point=[5.,5.],white_leg=True))
    assert b.active==[3.,3.5]
    b.dispatched(dict(r32_kind='interior_frontier'))
    assert b.active is None


def test_64_legs_remain_blocked_and_tour_uses_connected_route_cost():
    cells=np.zeros((120,120),np.int8)
    for r in range(8):
        for c in range(8):
            cells[15+r*10:17+r*10,15+c*10:17+c*10]=100
    # Build the same conservative footprint margin as the live SLAM route.
    from scipy.ndimage import distance_transform_edt
    safe=(cells==0)&(distance_transform_edt(np.pad(cells==0,1))[1:-1,1:-1]*.1>=.32)
    grid=Grid(cells,.1,(0.,0.,0.))
    pts=[(1.,1.),(10.5,1.),(10.5,10.5),(1.,10.5)]
    robot=(.6,5.5)
    distances=travel_distances(safe,*grid.index(*robot),.1)
    opts=[(float(distances[grid.index(*xy)]),dict(x=xy[0],y=xy[1],normal_odom=i*math.pi/2))
          for i,xy in enumerate(pts)]
    result=short_tour(opts,grid,safe,300.)
    assert result and result['box_tour_faces']==4
    assert (cells==100).sum()==64*4
    # A separating barrier cannot be crossed by the route optimizer.
    safe[:,60]=False
    distances=travel_distances(safe,*grid.index(*robot),.1)
    opts=[(float(distances[grid.index(g['x'],g['y'])]),g) for _,g in opts]
    assert short_tour(opts,grid,safe,300.)['box_tour_faces']==2


def test_saved_white_leg_is_selectable_after_box_visit():
    m=PrintMemory();m.defer_white=True
    for t in (1.6,1.8,2.):
        m.observe(dict(stamp=t,point=[4.,3.],observer=[2.,3.],white_leg=True),t)
    grid=Grid(np.zeros((90,90),np.int8),.1,(0.,0.,0.))
    result=m.choose(grid,grid,(2.,3.,0.),(0.,0.,0.),{},60.,lambda _:False,
                    white_filter=lambda _:True)
    assert result['white_leg'] and result['print_odom']==[4.,3.]
    assert result['print_route_m']>0


def test_named_marker_early_return_preserves_face_revisit():
    from concurrent.futures import Future
    from osp_mission.mission import Mission
    s=CenterWall();s.dispatched(face(),1.)
    cleared=[]
    owner=SimpleNamespace(action_generation=1,kind='nav',state='running',goal_handle=object(),
                          cancel_started=1.,cancel_reason='registered_marker',now=lambda:5.,started=0.,
                          hybrid=SimpleNamespace(schedule=s),contact_inspection=None,contact_pending=False,
                          marker_look=SimpleNamespace(clear=cleared.append),
                          face_look=SimpleNamespace(clear=cleared.append))
    result=Future();result.set_result(SimpleNamespace(status=5))
    Mission.completed(owner,result)
    assert len(s.continuity.suspended)==1
    assert not s.history and owner.scan_after is False
    assert owner.plan_after==5.
