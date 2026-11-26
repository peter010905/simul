import math
from types import SimpleNamespace
import numpy as np
from osp_mission.wall_lap import WallLap, measured_perimeter
from osp_mission._r32_navigation_core import Grid
from osp_mission.center_wall import CenterWall
from osp_mission.hybrid import Hybrid
from osp_mission.auto_chair_strip import chair_returns


def room():
    cells=np.zeros((90,130),np.int8)
    cells[:3,:]=100;cells[-3:,:]=100;cells[:,:3]=100;cells[:,-3:]=100
    cells[40:49,60:69]=100
    return Grid(cells,.1,(0.,0.,0.))


def test_coarse_waypoints_are_safe_and_do_not_target_box_faces():
    g=room();xy=(3.,3.,0.)
    safe,dist=g.reachable(xy[:2],.32)
    lap=WallLap()
    choice=lap.choose(g,dist,xy,(0.,0.,0.),9.,600.,lambda _:False)
    assert choice and choice['wall_lap'] and not choice['face_inspection']
    assert choice['scan_angle']==0.
    assert safe[g.index(choice['x'],choice['y'])]
    assert np.isfinite(dist[g.index(choice['x'],choice['y'])])
    pts,floor,_,_=measured_perimeter(g,xy)
    assert not np.any((pts[:,0]>=55)&(pts[:,0]<=74)&(pts[:,1]>=35)&(pts[:,1]<=54))


def test_chair_row_is_boundary_without_removing_collision_cells():
    g=room()
    for c in range(4,125,4):
        g.cells[12:13,c:c+2]=100
    before=g.cells.copy()
    assert chair_returns(g).any()
    p=measured_perimeter(g,(3.,3.))
    assert p is not None and p[3]>0
    assert not p[1][5,50]
    np.testing.assert_array_equal(before,g.cells)
    s=CenterWall()
    s.chair_surface_contains=lambda point:point[1]<1.6
    assert s.chair_surface_contains((3.,1.3))


def test_unreachable_boundary_waits_safely_until_the_overall_budget():
    g=room();dist=np.full(g.cells.shape,np.inf);lap=WallLap()
    assert lap.choose(g,dist,(3.,3.),(0.,0.,0.),10.,600.,lambda _:False) is None
    assert lap.choose(g,dist,(3.,3.),(0.,0.,0.),23.,600.,lambda _:False) is None
    assert lap.phase=='perimeter' and lap.reason=='waiting_for_safe_wall_entry'
    assert lap.choose(g,dist,(3.,3.),(0.,0.,0.),181.,600.,lambda _:False) is None
    assert lap.phase=='interior'


def test_loop_progress_requires_actual_travel_and_distinct_boundary_directions():
    lap=WallLap();lap.last_support=1.
    for i in range(8):
        goal=dict(wall_lap=True,lap_pose_odom=[0.,0.],lap_normal_odom=i*math.pi/2)
        lap.dispatched(goal)
        lap.position(0.,0.,i)
    assert lap.phase=='perimeter'  # Commands and in-place turns cannot complete a lap.
    lap.choose(room(),np.full((90,130),np.inf),(3.,3.),(0.,0.,0.),181.,600.,lambda _:False)
    assert lap.phase=='interior' and lap.reason=='perimeter_time_budget'


def test_marker_detour_keeps_progress_and_white_cues_wait():
    lap=WallLap();lap.path=8.;lap.entry=(1.,1.)
    lap.dispatched(dict(wall_lap=True,lap_pose_odom=[2.,1.],lap_normal_odom=0.))
    lap.interrupted()
    lap.position(10.,10.,20.)
    assert lap.path==8.
    h=Hybrid();h.schedule=SimpleNamespace(wall_lap=lap,continuity=SimpleNamespace(protecting=lambda _:False))
    h.memory.tracks=[dict(stamp=10.,count=3,white_leg=True)]
    owner=SimpleNamespace(now=lambda:10.,started=0.,get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000)))
    assert not h.has_fresh_print(owner)
    h.memory.tracks.append(dict(stamp=10.,count=3,white_leg=False))
    assert h.has_fresh_print(owner)


def test_after_perimeter_only_markers_can_request_an_ordinary_wall(monkeypatch):
    g=room();s=CenterWall();s.wall_lap.finish('test_completed')
    monkeypatch.setattr(s.interior_priority,'choose',lambda *a:None)
    monkeypatch.setattr(s.survey,'choose',lambda *a,**kw:None)
    result=s.decide(g,g,(3.,3.,0.),(0.,0.,0.),np.zeros((4,)+g.cells.shape,bool),200.,600.,lambda _:False)
    assert result is None and s.reason=='interior_tasks_temporarily_unavailable'


def test_marker_detours_do_not_extend_perimeter_budget():
    lap=WallLap()
    lap.dispatched(dict(focus_id=42))
    lap.expire(181.,600.)
    assert lap.phase=='interior'
    assert lap.reason=='perimeter_time_budget'


def test_an_unknown_edge_next_to_robot_cannot_hide_a_measured_wall():
    cells=np.full((100,100),-1,np.int8)
    cells[10:90,10:90]=0
    cells[10,10:90]=100
    cells[50:90,55:57]=-1
    g=Grid(cells,.1,(0.,0.,0.))
    safe,dist=g.reachable((5.,5.),.32)
    lap=WallLap()
    goal=lap.choose(g,dist,(5.,5.),(0.,0.,0.),9.,600.,lambda _:False)
    assert goal and goal['wall_entry']
    assert safe[g.index(goal['x'],goal['y'])]
    assert lap.phase=='perimeter'
    assert goal['y']<3.
