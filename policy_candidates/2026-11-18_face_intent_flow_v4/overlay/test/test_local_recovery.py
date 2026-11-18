from concurrent.futures import Future
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

import pytest
from nav2_msgs.action import NavigateToPose

from osp_mission.local_recovery import mark_departures
from osp_mission.mission import Mission
from osp_mission.recovery_control import RecoveryFlow, try_retreat


def mission():
    events = []
    clock = [10.]
    flow = RecoveryFlow()
    node = NS(navigation_flow=flow, kind='nav', state='running', cancel_reason='',
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000)),
              scan_stamp=10., now=lambda: clock[0], destination=(2., 1.), excluded=[], recovery_pending=False,
              last_recovery=-100., rear_clear=True, backup=NS(server_is_ready=lambda: True),
              send=lambda client, goal, kind: events.append((kind, goal)),
              zero=lambda: events.append('zero'), get_logger=lambda: NS(warning=lambda msg: None))
    return node, flow, clock, events


def fail(node, code=104, status=6):
    future = Future()
    future.set_result(NS(status=status, result=NS(error_code=code)))
    Mission.completed(node, future)


def test_recorded_stall_pattern_survives_goal_replacement_and_backs_only_once():
    node, flow, clock, events = mission()
    for i in range(11):
        clock[0] += 9
        flow.local.observe(.004 * (i % 2), 0., clock[0])
        goal = dict(x=2.+i, y=1., departure=(.40, 0.))
        flow.selected(goal, (0., 0., 0.))
        flow.begin_navigation()
        fail(node)
        if node.recovery_pending:
            assert try_retreat(node, flow)
    assert len(events) == 1
    kind, goal = events[0]
    assert kind == 'backup' and goal.target.x == pytest.approx(.20)
    assert goal.speed == pytest.approx(.08)
    assert flow.local.retry_after == clock[0] + 15.
    assert flow.blocked(dict(x=9., y=3., departure=(.40, .06)), (0., 0., 0.), clock[0])
    assert not flow.blocked(dict(x=9., y=3., departure=(0., -.40)), (0., 0., 0.), clock[0])
    flow.local.observe(-.20, 0., clock[0]+1.)
    assert flow.local.retreat_used
    flow.local.observe(-.50, 0., clock[0]+2.)
    assert not flow.local.retreat_used and flow.local.failures == 0


def test_rear_block_or_missing_tf_never_sends_a_retreat():
    node, flow, clock, events = mission()
    flow.selected(dict(x=1., y=0.), (0., 0., 0.))
    fail(node)
    assert not node.recovery_pending
    for _ in range(2):
        flow.local.observe(0., 0., clock[0])
        fail(node)
    node.rear_clear = False
    assert try_retreat(node, flow) and events == ['zero']
    clock[0] += 2
    node.rear_clear = True
    assert not try_retreat(node, flow) and events == ['zero']


@pytest.mark.parametrize('code,reason,status', [(208,'',6), (102,'',6), (0,'',4),
                                                 (0,'processed_camera_stale',5), (0,'marker_hint',5)])
def test_planning_sensor_marker_and_arrival_do_not_request_physical_escape(code, reason, status):
    node, flow, clock, events = mission()
    node.active_goal = NavigateToPose.Goal()
    node.sensor_interruptions = 0
    node.cancel_reason = reason
    flow.local.observe(0., 0., clock[0])
    fail(node, code, status)
    assert flow.local.failures == 0 and not node.recovery_pending


def test_controller_tree_on_actual_navigation_goal_has_replanning_and_no_autonomous_spin():
    node, flow, clock, events = mission()
    node.action_generation = 0
    node.accepted = lambda _: None
    sent = []
    def capture(goal, **kwargs):
        sent.append(goal)
        return Future()
    Mission.send(node, NS(send_goal_async=capture), NavigateToPose.Goal(), 'nav')
    tree = ET.parse(sent[0].behavior_tree)
    assert tree.find('.//ComputePathToPose') is not None and tree.find('.//FollowPath') is not None
    assert tree.find('.//Spin') is None and tree.find('.//BackUp') is None


def test_different_goals_sharing_a_doorway_get_the_same_departure():
    prev = [-9999, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    goals = [dict(cell=(0, 9)), dict(cell=(0, 11))]
    mark_departures(goals, prev, (1,12), .1, (0.,0.), (.05,.05))
    assert goals[0]['departure'] == goals[1]['departure'] == pytest.approx((.45,.05))

def test_stale_rear_scan_cannot_start_backup_even_if_last_report_was_clear():
    node, flow, _, events = mission()
    node.scan_stamp = 9.
    node.recovery_pending = True
    assert not try_retreat(node, flow) and not events


def test_single_blocked_exit_can_request_a_retreat_when_no_other_goal_remains():
    node, flow, clock, _ = mission()
    flow.local.observe(0., 0., clock[0])
    flow.selected(dict(x=1., y=0.), (0., 0., 0.))
    fail(node)
    assert not node.recovery_pending
    assert flow.local.request_retreat(clock[0])
    assert not flow.local.request_retreat(clock[0] + 2.)
    assert flow.local.request_retreat(clock[0] + 15.)


def test_single_exit_keeps_original_untried_goals_and_marker_override():
    _, flow, clock, _ = mission()
    flow.local.observe(0., 0., clock[0])
    failed = dict(x=1., y=0., departure=(.4, 0.))
    onward = dict(x=3., y=1., departure=(.4, .01))
    side = dict(x=0., y=4., departure=(0., .4))
    tf = (0., 0., 0.)
    flow.selected(failed, tf)
    flow.record_navigation_result(6, '', clock[0])
    flow.local.failed(flow.departure, 'no_progress', clock[0])
    assert flow.filter_goals([failed, onward], tf, clock[0]) == [onward]
    assert not flow.blocked(onward, tf, clock[0]) and flow.blocked(failed, tf, clock[0])
    assert flow.filter_goals([failed, onward, side], tf, clock[0]) == [side]
    assert flow.blocked(onward, tf, clock[0])


def test_real_heading_alignment_is_bounded_and_oscillation_cannot_extend_it():
    from math import radians
    for improving in (True, False):
        _, flow, _, _ = mission()
        flow.local.observe(0., 0., 0.)
        flow.selected(dict(x=2.,y=0.,departure=(.4,0.)), (0.,0.,0.))
        flow.begin_navigation()
        for t in range(21):
            flow.local.observe(0.,0.,t)
            flow.observe_position(0.,0.,t)
            yaw = 170 - t * 6 if improving else 100 + 10*(t % 2)
            flow.observe_heading(radians(yaw),t)
            if t == 12:
                assert flow.stalled(t) == (not improving)
        assert flow.stalled(20.)
