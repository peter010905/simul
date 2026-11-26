"""Synthetic map and mission-lifecycle regressions; these tests never drive hardware."""
from concurrent.futures import Future
import math
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest

from osp_mission._r32_navigation_core import Grid, connected_nav_cells
from osp_mission.center_wall import CenterWall
from osp_mission.crowded_navigation import CommonRoutes, CrowdedFlow
from osp_mission.mission import Mission


def maps(size=80, resolution=.05):
    cells = np.zeros((size,size), dtype=np.int16)
    cells[[0,-1],:] = 100
    cells[:,[0,-1]] = 100
    return Grid(cells,resolution,(0.,0.,0.)), Grid(cells.copy(),resolution,(0.,0.,0.))


def goal(grid, row, col, **kwargs):
    x,y = grid.world(row,col)
    return dict(x=float(x),y=float(y),yaw=0.,features=np.zeros(5),**kwargs)


def choose(goals, weights):
    return goals[0] if goals else None


def test_two_independently_reachable_masks_can_have_no_common_route():
    grid,nav = maps(100)
    # Each map has a different opening in the same divider.
    grid.cells[18:82,50] = 100
    nav.cells[:,50] = 100
    nav.cells[25:75,50] = 0
    pose = (*grid.world(50,20),0.)
    target = goal(grid,50,80)
    reachable,_ = grid.reachable(pose[:2],.32)
    assert reachable[50,80]
    assert connected_nav_cells(nav,pose[:2])[50,80]
    routes = CommonRoutes(grid,nav,pose)
    assert not routes.allows(target)


def test_shared_dogleg_corridor_is_allowed_without_line_of_sight():
    grid,nav = maps(100)
    grid.cells[1:72,50] = 100
    nav.cells[:] = grid.cells
    pose = (*grid.world(50,20),0.)
    target = goal(grid,50,80)
    routes = CommonRoutes(grid,nav,pose)
    assert routes.allows(target)
    assert routes.distance[50,80] > math.dist(pose[:2],(target['x'],target['y']))


@pytest.mark.parametrize('width,allowed', [(10,False),(22,True)])
def test_original_32cm_clearance_is_retained_at_narrow_passage(width,allowed):
    grid,nav = maps(100)
    grid.cells[:,48:52] = 100
    grid.cells[50-width//2:50+width//2,48:52] = 0
    nav.cells[:] = grid.cells
    routes = CommonRoutes(grid,nav,(*grid.world(50,20),0.))
    assert routes.allows(goal(grid,50,80)) == allowed


@pytest.mark.parametrize('cost', [-1,99,100])
def test_unknown_or_collision_costs_cannot_be_used_as_a_bridge(cost):
    grid,nav = maps()
    nav.cells[:,40] = cost
    routes = CommonRoutes(grid,nav,(*grid.world(40,20),0.))
    assert not routes.allows(goal(grid,40,60))


def test_diagonal_contact_cannot_cut_a_corner():
    grid,nav = maps(30,.1)
    nav.cells[:] = 100
    nav.cells[10,10] = nav.cells[11,11] = 0
    routes = CommonRoutes(grid,nav,(*grid.world(10,10),0.))
    assert not routes.allows(goal(grid,11,11))


def test_projection_supports_rotated_origins_and_different_resolutions():
    grid,_ = maps(40,.1)
    # Same physical square, Nav2 coordinates rotated 90 degrees.
    nav = Grid(np.zeros((80,80),dtype=np.int16),.05,(4.,0.,math.pi/2))
    routes = CommonRoutes(grid,nav,(*grid.world(15,15),0.))
    assert routes.allows(goal(grid,25,25))
    point = grid.world(25,25)
    nr,nc = nav.index(*point)
    nav.cells[nr-2:nr+3,nc-2:nc+3] = 100
    assert not CommonRoutes(grid,nav,(*grid.world(15,15),0.)).allows(goal(grid,25,25))


def test_reposition_uses_safe_side_when_rear_is_blocked():
    grid,nav = maps(100)
    pose = (*grid.world(50,40),0.)
    # Wall behind the robot, front left side remains open.
    grid.cells[30:70,30:32] = 100
    nav.cells[:] = grid.cells
    routes = CommonRoutes(grid,nav,pose)
    alternative = routes.reposition(lambda g:False,[],preferred_heading=0.)
    assert alternative is not None and routes.allows(alternative)
    assert .45 <= math.dist(pose[:2],(alternative['x'],alternative['y'])) <= 1.
    assert routes.clearance[alternative['cell']] >= .32
    assert alternative['crowded_reposition'] and alternative['scan_angle']==0.
    assert not any(k in alternative for k in ('approach_id','focus_id','surface_map'))


def test_reposition_moves_toward_opening_instead_of_returning_into_dead_end():
    grid,nav = maps(120)
    # U is open to the west. The robot already backed from x=3.5 to x=3.1.
    grid.cells[40:80,85:88]=100
    grid.cells[38:41,45:88]=100
    grid.cells[79:82,45:88]=100
    nav.cells[:]=grid.cells
    pose=(3.1,3.025,0.)
    flow=CrowdedFlow()
    for i,x in enumerate((3.5,3.3,3.1)):flow.observe_position(x,3.025,float(i))
    routes=CommonRoutes(grid,nav,pose)
    alternative=routes.reposition(lambda g:False,flow.reposition_visits((0.,0.,0.)),math.pi)
    assert routes.allows(alternative)
    assert alternative['x']<pose[0]-.4


def test_start_exception_never_rewrites_maps_or_opens_nav2_obstacles():
    grid,nav = maps()
    pose = (*grid.world(40,40),0.)
    grid.cells[40,40] = nav.cells[40,40] = 100
    before = grid.cells.copy(),nav.cells.copy()
    routes = CommonRoutes(grid,nav,pose,.30)
    assert not routes.allows(goal(grid,40,60))
    assert np.array_equal(grid.cells,before[0]) and np.array_equal(nav.cells,before[1])


def test_missing_shared_costmap_is_explicit_and_does_not_select_unchecked_goal():
    flow = CrowdedFlow()
    target = dict(x=1.,y=1.)
    assert flow.resolve(target,[target],None,(0.,0.,0.),10.,None,choose) is None
    assert flow.report()['planning_reason']=='waiting_for_shared_costmap'


def test_cross_goal_failure_survives_watchdog_reset_and_clears_on_real_progress():
    flow = CrowdedFlow()
    for now in (10.,20.):
        flow.begin_navigation()
        flow.observe_position(1.,1.,now)
        flow.selected(dict(x=2.,y=2.),(0.,0.,0.))
        flow.record_navigation_result(5,'navigation_stalled',now)
    assert flow.escape_anchor==(1.,1.)
    flow.begin_navigation()
    flow.observe_position(1.01,1.01,21.)
    assert flow.escape_anchor is not None
    flow.observe_position(1.5,1.,22.)
    assert flow.escape_anchor is None


@pytest.mark.parametrize('reason', ['scan_stale','registered_marker','readable_marker','near_pass_inspection'])
def test_intentional_or_sensor_cancellation_does_not_trigger_escape(reason):
    flow = CrowdedFlow()
    flow.observe_position(0.,0.,0.)
    for t in range(5):flow.record_navigation_result(5,reason,float(t))
    assert flow.escape_anchor is None


def test_survey_cannot_repeat_without_measured_translation():
    flow = CrowdedFlow()
    assert not flow.survey_allowed()
    flow.observe_position(1.,1.,0.)
    assert flow.survey_allowed() and not flow.survey_allowed()
    flow.observe_position(1.01,1.01,5.)
    assert not flow.survey_allowed()
    flow.observe_position(1.5,1.,10.)
    assert flow.survey_allowed()


def test_unreachable_selected_goal_is_replaced_by_common_safe_choice():
    grid,nav = maps(100)
    nav.cells[:,50] = 100
    routes = CommonRoutes(grid,nav,(*grid.world(50,20),0.))
    bad,good = goal(grid,50,80),goal(grid,70,20)
    flow = CrowdedFlow()
    assert flow.resolve(bad,[bad,good],routes,(0.,0.,0.),0.,None,choose) is good


def test_no_viewpoints_uses_local_reposition_without_marking_observation_complete():
    grid,nav = maps()
    routes = CommonRoutes(grid,nav,(*grid.world(40,40),0.))
    flow = CrowdedFlow()
    selected = flow.resolve(None,[],routes,(0.,0.,0.),0.,None,choose)
    assert routes.allows(selected) and selected['crowded_reposition']
    assert flow.report()['planning_reason']=='local_reposition'


def test_verified_marker_observation_keeps_priority_over_escape_reposition():
    grid,nav = maps()
    pose = (*grid.world(40,40),0.)
    routes = CommonRoutes(grid,nav,pose)
    flow = CrowdedFlow()
    flow.escape_anchor = pose[:2]
    selected = goal(grid,40,40,stationary=True,approach_id=103)
    assert flow.resolve(selected,[selected],routes,(0.,0.,0.),10.,None,choose) is selected


def test_empty_plan_survey_consumes_one_bounded_turn_in_actual_mission():
    node = NS(marker_look=None,face_look=None,crowded_survey_angle=math.pi/3,
              edge_mode=True,inspection_camera=None,map=None)
    assert Mission.observation_turn(node)==pytest.approx(math.pi/3)
    assert Mission.observation_turn(node)==0.


def test_failed_surface_dispatch_expires_while_successful_budget_is_preserved():
    schedule = CenterWall()
    anchor = np.array([1.,1.])
    surface = dict(surface_map=[1.,1.],surface_odom=anchor,normal_odom=0.)
    # Actual goal keys expected by the scheduler.
    schedule.phase='center'
    schedule.dispatched(surface,10.)
    assert schedule.attempts('center',anchor,0.,.5)==[10.]
    schedule.navigation_result(6,'',20.)
    schedule.elapsed=64.
    assert schedule.attempts('center',anchor,0.,.5)==[10.]
    schedule.elapsed=65.
    assert schedule.attempts('center',anchor,0.,.5)==[]
    schedule.dispatched(surface,70.)
    schedule.navigation_result(4,'',71.)
    schedule.elapsed=200.
    assert schedule.attempts('center',anchor,0.,.5)==[70.]


def test_sensor_pause_keeps_surface_attempt_for_resumed_success():
    schedule = CenterWall()
    anchor = np.array([1.,1.])
    schedule.phase='center'
    schedule.dispatched(dict(surface_map=[1.,1.],surface_odom=anchor,normal_odom=0.),10.)
    schedule.navigation_result(5,'scan_stale',20.)
    assert schedule.pending_attempt==10.
    schedule.navigation_result(4,'',30.)
    schedule.elapsed=100.
    assert schedule.attempts('center',anchor,0.,.5)==[10.]


@pytest.mark.parametrize('status', [4,5,6])
def test_only_successful_navigation_consumes_permanent_visit_exclusion(status):
    flow = CrowdedFlow()
    flow.selected(dict(x=1.,y=2.),None)
    node = NS(state='running',kind='nav',cancel_reason='navigation_stalled' if status==5 else '',
              goal_handle=object(),started=0.,now=lambda:10.,action_start=0.,plan_after=0.,
              destination=(1.,2.),excluded=[],navigation_flow=flow,exploration_memory=Mock(),
              marker_look=Mock(),face_look=Mock(),get_logger=lambda:Mock())
    future = Future()
    future.set_result(NS(status=status))
    Mission.completed(node,future)
    assert node.excluded==([(1.,2.)] if status==4 else [])
    assert flow.blocked(dict(x=1.,y=2.),None,11.) == (status!=4)
