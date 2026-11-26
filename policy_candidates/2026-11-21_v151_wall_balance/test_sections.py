import math
import numpy as np
from osp_mission._r32_navigation_core import Grid
from osp_mission.center_wall import CenterWall
from osp_mission.wall_sections import WallSections

def wall(x,y):
    return (0.,dict(x=x,y=y,surface_odom=[x+0.7,y],normal_odom=math.pi,r32_kind='wall_surface'))

def test_unseen_quiet_section_earns_a_turn_without_a_frontier_or_marker():
    scheduler=WallSections()
    g=Grid(np.zeros((80,80)),.1,(0,0,0))
    distances=np.ones((80,80))
    local,quiet=wall(1.,1.),wall(5.,6.)
    assert scheduler.choose([local,quiet],distances,g,0,600) is None
    scheduler.dispatched(local[1],55)
    selected=scheduler.choose([local,quiet],distances,g,61,600)
    assert selected['surface_odom']==quiet[1]['surface_odom']
    scheduler.dispatched(selected,61)
    assert scheduler.choose([local,quiet],distances,g,95,600,True) is None
    assert scheduler.choose([local,quiet],distances,g,176,600,True) is None
    scheduler.dispatched(dict(r32_kind='interior_surface'),177)
    assert scheduler.choose([local,quiet],distances,g,178,600,True) is not None

def test_no_fresh_view_or_no_time_or_disconnected_returns_no_extra_goal():
    scheduler=WallSections();g=Grid(np.zeros((40,40)),.1,(0,0,0))
    distances=np.ones_like(g.cells);goals=[wall(2.,1.)]
    scheduler.choose(goals,distances,g,0,600)
    assert scheduler.choose([],distances,g,80,600) is None
    distances[:]=math.inf
    assert scheduler.choose(goals,distances,g,80,600) is None
    distances[:]=1.
    assert scheduler.choose(goals,distances,g,598,600) is None

def test_native_selector_takes_wall_with_fully_known_floor_and_fresh_box_then_releases():
    gridcells=np.zeros((90,110),dtype=np.int8)
    gridcells[[1,-2],:]=100;gridcells[:,[1,-2]]=100;gridcells[40:54,42:56]=100
    grid=Grid(gridcells,.05,(0.,0.,0.));cost=Grid(np.zeros_like(gridcells),.05,grid.origin)
    pose=(1.2,1.3,0.);tf=(0.,0.,0.);s=CenterWall()
    seen=np.zeros((4,)+gridcells.shape,bool)
    def choose(t,veto=lambda _:False,probe=False):
        return s.decide(grid,cost,pose,tf,seen,t,600,veto,choices=(),probe_only=probe)
    assert choose(1)['r32_kind']=='interior_surface'
    assert 'wall_section' not in choose(70,probe=True)
    w=choose(70)
    assert w['r32_kind']=='wall_surface' and 'wall_section' in w
    assert not seen.any()  # Dispatch cannot manufacture camera evidence.
    s.dispatched(w,70)
    assert choose(71)['r32_kind']=='interior_surface'
    assert choose(132,lambda g:g['r32_kind']=='wall_surface')['r32_kind']=='interior_surface'
    seen[:]=True
    assert choose(150) is None

def test_all_four_camera_deficits_receive_turns_with_continuous_fresh_boxes():
    scheduler=WallSections()
    scheduler.axes=0.
    scheduler.coverage={0:.9,1:.1,2:.05,3:0.}
    grid=Grid(np.zeros((80,80)),.1,(0,0,0))
    distance=np.ones_like(grid.cells)
    goals=[]
    for i in range(4):
        _,g=wall(1.+i,1.)
        g['normal_odom']=i*math.pi/2
        goals.append((0.,g))
    scheduler.choose(goals,distance,grid,0,600,True)
    directions=[]
    for t in (46,92,138,184):
        chosen=scheduler.choose(goals,distance,grid,t,600,True)
        assert chosen is not None
        directions.append(chosen['wall_direction'])
        scheduler.dispatched(chosen,t)
        scheduler.dispatched(dict(r32_kind='interior_surface'),t+1)
    assert set(directions)==set(range(4))
    assert directions[0]==3

def test_dispatch_does_not_increase_measured_camera_coverage():
    scheduler=WallSections();scheduler.axes=0.
    scheduler.coverage={0:.2}
    scheduler.dispatched(wall(2.,1.)[1],60)
    assert scheduler.coverage=={0:.2}
