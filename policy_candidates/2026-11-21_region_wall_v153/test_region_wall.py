import math
from types import SimpleNamespace
import numpy as np
from osp_mission.region_wall import RegionWall, coverage_witnesses, measure, strips


def wall(x, walk=2, component=1):
    return dict(wall_line=(component,0),surface_odom=[x,0.],normal_odom=math.pi/2,
                wall_walk_m=walk, unseen_wall_m=.7,r32_kind='wall_surface')


def test_pick_reachable_long_strip_over_near_isolated_goal():
    policy=RegionWall()
    walls=[(1,wall(0,.5)), *((5,wall(x,5,2)) for x in range(5,10))]
    assert policy.choose(walls,20,600) is None
    goal=policy.choose(walls,91,600)
    assert goal['wall_line']==(2,0)
    policy.dispatched(goal,91)
    assert policy.choose(walls,98,600)['strip_reason']=='continue_wall_strip'
    assert policy.choose(walls,130,600) is None


def test_separate_disconnected_runs_and_no_walls():
    assert len(list(strips([(0,wall(x)) for x in (2,3,8,9)])))==2
    assert RegionWall().choose([],599,600) is None


def test_deadline_only_preempts_general_transit_once():
    p=RegionWall()
    p.choose([(2,wall(1))],20,600)
    p.dispatched(dict(r32_kind='interior_frontier'),20)
    assert not p.due_in_transit(89)
    assert p.due_in_transit(90)
    assert not p.due_in_transit(91)
    for goal in (dict(approach_id=7),dict(print_point=[0,1]),dict(focus_id=3)):
        p.dispatched(goal,20)
        assert not p.due_in_transit(90)


def test_measure_reprojects_camera_observation_after_map_origin_shift():
    grid=SimpleNamespace(cells=np.full((15,15),100),origin=(0,0,0),resolution=.1)
    seen=np.zeros((4,15,15),bool)
    samples={('interior',1,0,0,0):[(3,c) for c in range(3,8)]}
    pts=coverage_witnesses(samples,seen,grid,(1,2,0))
    grid.origin=(.1,0,0)
    seen[0,3,2:5]=True
    assert math.isclose(measure((pts,0),grid,(1,2,0),seen),.3)
    grid.cells[:]=-1
    assert measure((pts,0),grid,(1,2,0),seen) is None


def test_two_low_yield_views_defer_region_but_new_face_reopens():
    p=RegionWall()
    grid=SimpleNamespace(cells=np.full((20,20),100),origin=(0,0,0),resolution=.1)
    seen=np.zeros((4,20,20),bool)
    samples={('interior',1,0,0,0):[(3,c) for c in range(3,8)]}
    g=dict(r32_kind='interior_surface',surface_odom=[.5,.5],fitted_face=1,normal_odom=0)
    for t in (0,12):
        p.observe(samples,grid,(0,0,0),seen,t,[(0,g)])
        p.dispatched(g,t)
    p.observe(samples,grid,(0,0,0),seen,24,[(0,g)])
    assert p.deferred(g)
    assert not p.deferred(dict(g,normal_odom=math.pi/2))
    assert p.choose([(1,wall(1))],24,600)['strip_reason']=='region_low_yield'
    p.observe(samples,grid,(0,0,0),seen,101,[(0,g)])
    assert not p.deferred(g)


def test_real_scheduler_respects_veto_and_probe_only():
    from osp_mission import _r32_navigation_core as r32
    from osp_mission.center_wall import CenterWall
    grid=np.zeros((64,64),np.int8)
    grid[50,:]=100
    grid[51:,:]=-1
    slam=r32.Grid(grid,.1,(0.,0.,0.))
    cost=r32.Grid(np.zeros_like(grid),.1,(0.,0.,0.))
    def choose(veto,probe=False):
        return CenterWall().decide(slam,cost,np.array([3.,3.,0.]),(0.,0.,0.),
            np.zeros((4,64,64),bool),105.,600.,veto,probe_only=probe)
    allowed=choose(lambda _:False)
    assert allowed['region_wall'] and allowed['wall_walk_m']>0
    assert choose(lambda _:True) is None
    assert choose(lambda _:False,True) is None


def test_registered_approach_and_readable_focus_bypass_regions():
    from osp_mission.phased_hybrid import PhasedHybrid
    h=PhasedHybrid.__new__(PhasedHybrid)
    for key in ('approach_id','focus_id'):
        goal={key:32}
        assert h.select(None,goal,None,None,None,None) is goal
