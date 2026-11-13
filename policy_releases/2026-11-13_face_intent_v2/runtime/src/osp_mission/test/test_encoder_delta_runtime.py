import math

import pytest
from nav_msgs.msg import Odometry
from osp_mission.encoder_delta_twist import EncoderDelta


def reading(t,x,y,yaw=0.,reported=4.):
    value=Odometry();value.header.frame_id='odom';value.child_frame_id='base_link'
    value.header.stamp.sec=int(t);value.header.stamp.nanosec=round((t-int(t))*1e9)
    value.pose.pose.position.x=x;value.pose.pose.position.y=y
    value.pose.pose.orientation.z=math.sin(yaw/2);value.pose.pose.orientation.w=math.cos(yaw/2)
    value.twist.twist.linear.x=reported;value.twist.covariance[0]=.001
    return value


def test_backward_difference_uses_measured_pose_increment_and_original_stamp():
    source=EncoderDelta();assert source.update(reading(1.,0.,0.)) is None
    raw=reading(1.06,.024,0.);result=source.update(raw)
    assert result.twist.twist.linear.x==pytest.approx(.4)
    assert result.header==raw.header and result.twist.covariance[0]==.001
    assert raw.twist.twist.linear.x==4.


def test_reverse_motion_and_rotated_body_frame():
    source=EncoderDelta();source.update(reading(1.,0.,0.,math.pi/2))
    result=source.update(reading(1.02,0.,-.006,math.pi/2))
    assert result.twist.twist.linear.x==pytest.approx(-.3)
    assert abs(result.twist.twist.linear.y)<1e-10


def test_turn_wrap_uses_short_rotation():
    source=EncoderDelta();source.update(reading(1.,0.,0.,math.pi-.01))
    result=source.update(reading(1.02,-.006,0.,-math.pi+.01))
    assert result.twist.twist.linear.x==pytest.approx(.3)


def test_duplicate_and_out_of_order_do_not_poison_next_increment():
    source=EncoderDelta();source.update(reading(1.,0.,0.))
    assert source.update(reading(1.,5.,5.)) is None
    assert source.update(reading(.9,5.,5.)) is None
    assert source.update(reading(1.02,.006,0.)).twist.twist.linear.x==pytest.approx(.3)


def test_gap_or_changed_frame_starts_new_segment():
    source=EncoderDelta();source.update(reading(1.,0.,0.))
    assert source.update(reading(2.,1.,0.)) is None
    assert source.update(reading(2.02,1.006,0.)).twist.twist.linear.x==pytest.approx(.3)
    value=reading(2.04,1.012,0.);value.header.frame_id='other'
    assert source.update(value) is None


def test_invalid_pose_never_becomes_a_velocity():
    source=EncoderDelta();source.update(reading(1.,0.,0.))
    value=reading(1.02,math.nan,0.)
    assert source.update(value) is None
    value=reading(1.02,.006,0.);value.pose.pose.orientation.w=0.
    assert source.update(value) is None
