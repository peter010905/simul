from types import SimpleNamespace as NS
import numpy as np
import pytest
from osp_mission import _r32_navigation_core as r32
from osp_mission.hybrid import Hybrid,PrintMemory
from osp_mission.marker_directed_look import MarkerDirectedLook


def evidence(stamp):
    return dict(stamp=stamp,point=[3.,2.],observer=[.8,2.],size=18.)


def setup_grid():
    return r32.Grid(np.zeros((60,100),dtype=np.int8),.08,(0.,0.,0.))


def plan(mem,grid,cost,*,now=11.,pose=(.8,2.,0.),targets=None,veto=lambda g:False):
    return mem.choose(grid,cost,pose,(0,0,0),targets or {},now,veto)


def test_only_repeat_surface_moves_and_never_confirms_id():
    grid=setup_grid(); mem=PrintMemory()
    for s in (10.,10.,10.1):
        mem.observe(evidence(s),s)
    assert plan(mem,grid,grid) is None
    mem.observe(evidence(10.2),10.2)
    goal=plan(mem,grid,grid)
    assert goal and goal['x']>.8 and goal['yaw']==pytest.approx(0.)
    assert not {'focus_id','approach_id'} & goal.keys()
    assert plan(mem,grid,grid,targets={7:np.array([3.,2.,.5])}) is None
    assert plan(mem,grid,grid,veto=lambda g:True) is None
    assert plan(mem,grid,grid,now=30.) is None
    barrier=grid.cells.copy();barrier[:,12:15]=100
    assert plan(mem,grid,r32.Grid(barrier,.08,grid.origin)) is None
    mem.used.append((np.array([3.,2.]),11.))
    assert plan(mem,grid,grid) is None


def test_id_goals_win_and_missing_costmap_does_not_replace_flow():
    h=Hybrid(); goal={'focus_id':7}
    assert h.select(None,goal,None,None,None,None) is goal
    goal={'approach_id':103}
    assert h.select(None,goal,None,None,None,None) is goal
    owner=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=1e9)))
    flow={'x':1}
    assert h.select(owner,flow,None,None,None,None) is flow


def test_anonymous_arrival_uses_markers_camera_look_but_has_no_id():
    look=MarkerDirectedLook(NS()); look.event=lambda *args,**kwargs:None
    look.select(dict(print_point=[2.,0.]),(1.,0.,0.))
    assert look.pending['point_odom']==[3.,0.] and look.pending['id'] is None
    look.clear('bump_contact')
    assert look.pending is None


def test_actual_r32_corridor_returns_one_forward_goal_and_skips_blocked_end():
    g=setup_grid()
    g.cells[5,:]=100;g.cells[35,:]=100
    pose=(1.2,1.6,0.)
    reach=r32.nav_goal_reachability(g,g,pose,g.reachable(pose[:2],.30))
    goal=r32.corridor_waypoint(g,pose,reach,[],[],.45)
    assert goal[3]=='corridor' and goal[0]>pose[0]+1.2
    assert r32.corridor_waypoint(g,pose,reach,[],[np.array(goal[:2])],.45) is None


def test_new_mission_before_first_bumper_event():
    import rclpy
    from osp_mission.mission import Mission
    rclpy.init()
    try:
        node=Mission()
        try:
            assert node.contact_pending is False
            assert node.recovery_pending is False
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_live_adapter_dispatches_corridor_and_preserves_blocked_flow_fallback():
    from nav_msgs.msg import OccupancyGrid
    from osp_mission.navigation_flow import NavigationFlow
    grid=setup_grid()
    grid.cells[5,:]=100
    grid.cells[35,:]=100
    msg=OccupancyGrid()
    msg.header.frame_id='map';msg.header.stamp.sec=10
    msg.info.resolution=grid.resolution
    msg.info.height,msg.info.width=grid.cells.shape
    msg.info.origin.orientation.w=1.
    msg.data=grid.cells.flatten().tolist()
    clock=NS(now=lambda:NS(nanoseconds=10_000_000_000))
    owner=NS(get_clock=lambda:clock,map=msg,targets={},navigation_flow=NavigationFlow(),
             started=0.,now=lambda:10.,p=lambda k:{'duration':600.,'map_frame':'map'}[k],
             exploration_memory=NS(views=[]),excluded=[])
    hybrid=Hybrid()
    hybrid.drive.update_costmap(msg)
    pose=(1.2,1.6,0.)
    viewed=np.zeros((4,*grid.cells.shape),dtype=bool)
    fallback=dict(x=1.5,y=1.6)
    selected=hybrid.select(owner,fallback,grid.cells,viewed,pose,(0.,0.,0.))
    assert selected['r32_kind']=='corridor'
    owner.excluded.append([selected['x'],selected['y']])
    assert hybrid.select(owner,fallback,grid.cells,viewed,pose,(0.,0.,0.)) is fallback


def test_passed_box_view_is_a_single_camera_task_in_the_original_mission():
    from nav_msgs.msg import OccupancyGrid
    from osp_mission.navigation_flow import NavigationFlow
    cells=np.zeros((100,180),dtype=np.int8)
    cells[10,:]=cells[80,:]=100
    cells[46:59,57:70]=100
    msg=OccupancyGrid()
    msg.info.resolution=.05; msg.info.height,msg.info.width=cells.shape
    msg.info.origin.orientation.w=1.; msg.data=cells.flatten().tolist()
    msg.header.frame_id='map';msg.header.stamp.sec=10
    owner=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000)),map=msg,targets={},
             navigation_flow=NavigationFlow(),started=0.,now=lambda:10.,
             p=lambda k:{'map_frame':'map','duration':600.}[k],exploration_memory=NS(views=[]),excluded=[])
    h=Hybrid(); h.drive.update_costmap(msg); transform=(0.,0.,0.)
    viewed=np.zeros((4,*cells.shape),dtype=bool)
    ordinary=dict(x=2.,y=2.)
    corridor=h.select(owner,ordinary,cells,viewed,(2.1,2.,0.),transform)
    assert corridor['r32_kind']=='corridor'
    pose=(corridor['x'],corridor['y'],corridor['yaw'])
    g=h.select(owner,ordinary,cells,viewed,pose,transform)
    assert g['r32_kind']=='box_look' and g['stationary']
    look=MarkerDirectedLook(NS());look.event=lambda *a,**kw:None
    look.select(g,transform)
    assert look.pending['kind']=='passed_box' and look.pending['id'] is None
    h.seen_boxes.append((g['box_odom'],owner.now()))
    assert h.select(owner,ordinary,cells,viewed,pose,transform) is ordinary
