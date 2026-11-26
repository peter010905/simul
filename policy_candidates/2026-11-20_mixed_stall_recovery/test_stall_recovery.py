from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
from osp_mission.mission import Mission
from osp_mission.navigation_flow import NavigationFlow
from osp_mission.stall_recovery import StallPause


class PlanningReached(Exception):
    pass


class Owner(NS):
    def __getattr__(self, name):
        if name == 'map':
            raise PlanningReached()
        raise AttributeError(name)


def owner(reason='navigation_stalled', rear=True):
    flow = NavigationFlow()
    flow.selected(dict(x=1., y=2.), None)
    return Owner(state='running', targets={}, collision_count=0, started=1.,
                 now=lambda:100., p=lambda key:600., ready=lambda:True,
                 status=Mock(), goal_handle=object(), pending=False, kind='nav',
                 cancel_reason=reason, action_start=90., destination=(1.,2.),
                 excluded=[], navigation_flow=flow, surface_mode=True,
                 initial_scan=False, scan_after=False, plan_after=0.,
                 recovery_pending=False, spin_blocked_until=0.,
                 rear_clear=rear, backup=Mock(), last_recovery=-100.,
                 camera_diagnostics={"accepted":0}, tf=Mock(),
                 marker_look=NS(pending=None,clear=Mock()),
                 face_look=NS(pending=None,clear=Mock()),
                 resume_action=None, zero=Mock(), send=Mock(),
                 get_logger=lambda:Mock())


@pytest.mark.parametrize('status,reason', [(5,'navigation_stalled'),(6,''),(6,'action_timeout')])
def test_failed_navigation_uses_existing_safe_retreat_before_replanning(status,reason):
    node=owner(reason)
    future=Future();future.set_result(NS(status=status))
    Mission.completed(node,future)
    assert node.recovery_pending and not node.scan_after
    assert node.navigation_flow.blocked(dict(x=1.,y=2.),None,100.)
    Mission.tick(node)
    client,goal,kind=node.send.call_args.args
    assert client is node.backup and kind=='backup'
    assert goal.target.x==pytest.approx(.20) and goal.speed==pytest.approx(.08)


def test_rear_obstacle_and_backup_cooldown_prevent_retreat():
    for rear,cooldown in [(False,False),(True,True)]:
        node=owner(rear=rear)
        if cooldown:node.navigation_flow.backup_result(6,99.)
        future=Future();future.set_result(NS(status=5))
        Mission.completed(node,future)
        with pytest.raises(PlanningReached):Mission.tick(node)
        node.send.assert_not_called()


def test_brief_stall_needs_clear_signal_freshness_and_measured_rest():
    guard=StallPause()
    guard.update([3],0.)
    assert guard.paused and not guard.fatal(.1)
    guard.update([], .2)
    assert not guard.resume(.5,True)
    # A single clear packet cannot prove the signal remains clear.
    assert not guard.resume(1.3,True)
    for now in (1.4,1.7,2.0):
        guard.update([],now)
        recovered=guard.resume(now,True)
    assert recovered and not guard.paused and guard.recovered==1


def test_persistent_stall_and_critical_hazards_cannot_resume():
    for types in ([3],[3,2],[3,4]):
        guard=StallPause();guard.update(types,10.)
        assert guard.fatal(12.)
        assert not guard.resume(12.,True)
    guard=StallPause();guard.update([3],0.)
    for now in (.2,.7,1.2,1.7):
        guard.update([],now)
        assert not guard.resume(now,False)
    assert guard.paused


def test_mission_holds_until_cancel_result_and_clear_rest_then_checks_retreat():
    node=owner('motor_stall')
    node.motor_stall=StallPause()
    node.motor_stall.update([3],100.)
    node.cancel_action=Mock()
    node.stall_velocity=(0.,0.)
    node.wheel_stamp=100.
    node.sensor_fresh=lambda stamp:True
    assert Mission.hold_motor_stall(node)
    node.cancel_action.assert_called_once_with('motor_stall')
    node.send.assert_not_called()
    node.goal_handle=None
    node.motor_stall.update([],100.1)
    for t in (100.2,100.7,101.2):
        node.now=lambda:t
        node.motor_stall.update([],t)
        assert Mission.hold_motor_stall(node)
    assert node.recovery_pending and not node.motor_stall.paused
    node.zero.assert_called()


def test_stall_callback_cancels_immediately_and_stale_clear_cannot_release():
    from std_msgs.msg import Header
    node=owner('')
    node.motor_stall=StallPause()
    node.sensor_fresh=lambda stamp:False
    node.near_pass=NS(abort=Mock())
    node.cancel_action=Mock()
    msg=NS(header=Header(),detections=[NS(type=3)])
    Mission.on_hazard(node,msg)
    node.zero.assert_called_once()
    node.cancel_action.assert_called_once_with('motor_stall')
    assert node.motor_stall.paused and node.motor_stall.updated==100.
    node.now=lambda:101.
    msg.detections=[]
    Mission.on_hazard(node,msg)
    assert node.motor_stall.clear_since is None
    node.send.assert_not_called()


def test_supervisor_keeps_legacy_defaults_and_bounds_opt_in_stall_pause():
    from robot_session import BumpRecoveryGuard
    assert BumpRecoveryGuard().check(0.,[3])
    guard=BumpRecoveryGuard(StallPause())
    assert not guard.check(0.,[3])
    assert not guard.check(1.9,[3])
    assert guard.check(2.,[3])
    for danger in ([2],[4],[2,3],[4,3]):
        assert BumpRecoveryGuard(StallPause()).check(0.,danger)
