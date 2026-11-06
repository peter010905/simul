"""Regression for the native probe canceled at sim 9.0 after 7.88 wall seconds."""
from concurrent.futures import Future
from collections import Counter
from types import SimpleNamespace as Obj

import pytest
from rclpy.time import Time
from rclpy.clock import ClockType
from sensor_msgs.msg import CameraInfo

from osp_mission.mission import Mission


def clock_owner(seconds):
    return Obj(get_clock=lambda: Obj(now=lambda: Time(seconds=seconds,clock_type=ClockType.ROS_TIME)))


def test_slow_simulation_uses_message_time_not_wall_receipt():
    owner=clock_owner(9.)
    # Native probe: last processed camera stamp 7.8, received at wall +4.71;
    # cancellation at wall +7.88. 3.17 wall seconds are only 1.2 sim seconds.
    assert Mission.sensor_fresh(owner,7.8)
    assert not Mission.sensor_fresh(owner,6.9)
    assert not Mission.sensor_fresh(owner,None)
    assert not Mission.sensor_fresh(owner,10.)  # clock reset/future data


@pytest.mark.parametrize('fresh,elapsed,expected',[
    (True,2.759,[]),(False,2.759,['scan_stale_during_undock']),
    (True,66.,['undock_timeout'])])
def test_undock_ignores_camera_and_map_dropouts_but_keeps_scan_watchdog(fresh,elapsed,expected):
    canceled=[]
    owner=Obj(state='running',targets={},collision_count=0,started=1.,
        status=Obj(publish=lambda m:None),now=lambda:1.+elapsed,p=lambda key:600.,
        kind='undock',goal_handle=object(),pending=False,action_start=1.,
        scan_stamp=1.,sensor_fresh=lambda stamp:fresh,cancel_action=canceled.append)
    # No ready()/camera/TF/zero methods: undock must remain owned by Create 3.
    Mission.tick(owner)
    assert canceled==expected


@pytest.mark.parametrize('status,docked,fails',[(4,False,False),(4,True,True),(5,False,True),(6,True,True)])
def test_canceled_or_still_docked_action_cannot_report_success(status,docked,fails):
    reasons=[]
    owner=Obj(goal_handle=object(),state='running',kind='undock',cancel_reason='test',
        now=lambda:9.,action_start=6.241,finish=reasons.append,
        get_logger=lambda:Obj(info=lambda s:None))
    future=Future();future.set_result(Obj(status=status,result=Obj(is_docked=docked)))
    Mission.completed(owner,future)
    assert reasons==(['undock_failed'] if fails else [])
    assert owner.undock_diagnostics['status']==status
    assert owner.undock_diagnostics['cancel_reason']=='test'


def test_camera_heartbeat_survives_delayed_tf_without_crediting_surface():
    owner=clock_owner(7.85)
    owner.camera_diagnostics=Counter()
    owner.camera_stamp=None;owner.map=None;owner.latest_scan=None
    owner.observe_camera=lambda:Mission.observe_camera(owner)
    message=CameraInfo();message.width=960;message.k=[600.,0.,480.,0.,600.,360.,0.,0.,1.]
    message.header.stamp=Time(seconds=7.8).to_msg()
    Mission.on_camera(owner,message)
    assert owner.camera_stamp==pytest.approx(7.8)
    assert owner.pending_camera is message
    # Delayed/duplicate heartbeat cannot extend freshness or replace the queue.
    message.header.stamp=Time(seconds=7.).to_msg()
    Mission.on_camera(owner,message)
    assert owner.camera_stamp==pytest.approx(7.8)
