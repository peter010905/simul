from concurrent.futures import Future
from types import SimpleNamespace
import math

import pytest

from osp_mission.navigation_flow import NavigationFlow
from osp_mission.mission import Mission


def test_failed_goal_cooldown_expires_and_other_goals_remain_available():
    flow=NavigationFlow();goal={'x':1.,'y':2.}
    flow.selected(goal,(0.,0.,0.));flow.record_navigation_result(6,'',10.)
    alternative={'x':2.,'y':2.}
    assert flow.filter_goals([goal,alternative],(0.,0.,0.),69.)==[alternative]
    assert flow.filter_goals([goal,alternative],(0.,0.,0.),70.)==[goal,alternative]


def test_failed_goal_anchor_survives_a_map_frame_correction():
    flow=NavigationFlow();flow.selected({'x':1.,'y':2.},(0.,0.,0.))
    flow.record_navigation_result(5,'navigation_stalled',10.)
    assert flow.blocked({'x':-9.,'y':2.},(10.,0.,0.),20.)
    assert not flow.blocked({'x':1.,'y':2.},None,20.)


@pytest.mark.parametrize('reason', ['processed_camera_stale','scan_stale','marker_hint',
                                   'near_pass_inspection','near_pass_time_limit'])
def test_observation_and_sensor_cancellations_do_not_ban_valid_goals(reason):
    flow=NavigationFlow();goal={'x':1.,'y':2.};flow.selected(goal,None)
    flow.record_navigation_result(5,reason,10.)
    assert not flow.blocked(goal,None,10.)


def test_success_does_not_create_a_failure_and_expired_entries_are_not_fallbacks():
    flow=NavigationFlow();goal={'x':1.,'y':2.};flow.selected(goal,None)
    flow.record_navigation_result(4,'',10.);assert not flow.failures
    flow.record_navigation_result(6,'',11.)
    assert flow.filter_goals([goal],None,12.)==[]


def test_pose_noise_cannot_extend_stalled_navigation_forever():
    flow=NavigationFlow();flow.begin_navigation();flow.observe_position(0.,0.,10.)
    for t in range(11,18):flow.observe_position(.003, -.003,t)
    assert not flow.stalled(17.9) and flow.stalled(18.)
    flow.observe_position(.07,0.,18.)
    assert not flow.stalled(25.9) and flow.stalled(26.)


def test_navigation_resume_resets_watchdog_and_missing_pose_does_not_fake_progress():
    flow=NavigationFlow();flow.observe_position(0.,0.,1.)
    flow.begin_navigation();assert not flow.stalled(100.)
    flow.observe_position(math.nan,0.,100.)
    assert flow.progress_time is None


def test_real_camera_frame_settle_is_shorter_than_fixed_wait_but_not_immediate():
    flow=NavigationFlow();flow.begin_settle(10.,20)
    assert flow.waiting_for_frames(10.2,25)
    assert not flow.waiting_for_frames(10.5,25)
    flow.begin_settle(20.,30)
    assert flow.waiting_for_frames(21.,31)
    assert not flow.waiting_for_frames(21.5,31)


def test_failed_backup_has_a_cooldown_without_issuing_any_motion():
    flow=NavigationFlow();flow.backup_result(6,10.)
    assert not flow.backup_allowed(69.) and flow.backup_allowed(70.)


class PlanningReached(Exception):
    pass


def owner(active=False, angle=0., protected=False):
    events=[];clock=[100.]
    def planning(*args):
        raise PlanningReached()
    stub=SimpleNamespace(state='running',targets={},collision_count=0,started=1.,
        status=SimpleNamespace(publish=lambda message:None),now=lambda:clock[0],p=lambda key:600.,
        ready=lambda:True,goal_handle=object() if active else None,pending=False,
        plan_after=0.,recovery_pending=False,scan_after=not active,spin_blocked_until=0.,
        surface_mode=True,initial_scan=False,kind='nav',cancel_reason='',action_start=99.,
        marker_look=SimpleNamespace(pending={} if protected else None,clear=planning),
        face_look=SimpleNamespace(pending=None),spin=object(),camera_diagnostics={'accepted':10},
        near_pass=SimpleNamespace(poll=lambda:None,tick=lambda:False,request=lambda:events.append('wall_interrupt')),
        zero=lambda:events.append('zero'),cancel_action=lambda reason:events.append(reason),
        send=lambda *args:events.append('send'),observation_turn=lambda:angle,
        get_logger=lambda:SimpleNamespace(info=lambda *a:None,warning=lambda *a:None),
        navigation_flow=NavigationFlow())
    return stub,events,clock


def test_actual_mission_tick_does_not_interrupt_transit_for_a_wall_glance():
    stub,events,_=owner(active=True)
    Mission.tick(stub)
    assert not events
    stub.navigation_flow.allow_mid_route_wall_inspection=True
    Mission.tick(stub)
    assert events==['wall_interrupt']


def test_actual_mission_tick_advances_after_a_zero_turn_without_stop_or_wait():
    stub,events,_=owner()
    with pytest.raises(PlanningReached):Mission.tick(stub)
    assert not events and stub.plan_after==0.
    assert stub.navigation_flow.skipped_zero_turns==1


def test_zero_turn_for_marker_inspection_still_waits_for_readable_frames():
    stub,events,clock=owner(protected=True)
    Mission.tick(stub)
    assert events==['zero'] and stub.navigation_flow.settle is not None
    clock[0]=100.4;stub.camera_diagnostics['accepted']=20
    Mission.tick(stub);assert events==['zero','zero']
    clock[0]=100.5
    with pytest.raises(PlanningReached):Mission.tick(stub)


def test_stalled_navigation_is_cancelled_but_pending_marker_confirmation_is_preserved():
    stub,events,_=owner(active=True)
    stub.navigation_flow.observe_position(0.,0.,90.)
    Mission.tick(stub)
    assert events==['navigation_stalled','zero'] and stub.navigation_flow.stalls_cancelled==1
    stub,events,_=owner(active=True);stub.cancel_reason='marker_hint'
    stub.navigation_flow.observe_position(0.,0.,90.)
    Mission.tick(stub);assert not events


def test_actual_callback_bans_failed_goal_but_not_a_sensor_pause():
    for reason,blocked in [('navigation_stalled',True),('processed_camera_stale',False)]:
        flow=NavigationFlow();flow.selected({'x':1.,'y':2.},None)
        stub=SimpleNamespace(state='running',kind='nav',goal_handle=object(),cancel_reason=reason,
            now=lambda:100.,destination=(1.,2.),excluded=[],navigation_flow=flow,
            active_goal=object(),sensor_interruptions=0,
            get_logger=lambda:SimpleNamespace(warning=lambda *a:None))
        f=Future();f.set_result(SimpleNamespace(status=5));Mission.completed(stub,f)
        assert flow.blocked({'x':1.,'y':2.},None,101.) is blocked
