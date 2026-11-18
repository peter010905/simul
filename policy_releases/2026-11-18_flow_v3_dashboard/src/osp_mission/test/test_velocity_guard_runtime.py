import math

import pytest
from nav_msgs.msg import Odometry
from osp_mission.calibrated_wheel_twist import correct,plausible


def reading(speed,lateral=0.):
    message=Odometry();message.child_frame_id='base_link'
    message.header.stamp.sec=20;message.header.stamp.nanosec=100
    message.twist.twist.linear.x=speed;message.twist.twist.linear.y=lateral
    message.twist.covariance[0]=.001
    return message


@pytest.mark.parametrize('speed',[-.65,-.46,0.,.46,.65])
def test_forward_reverse_and_transient_readings_within_margin_remain_unmodified(speed):
    message=correct(reading(speed),.78)
    assert plausible(message,.75)
    assert message.twist.twist.linear.x==pytest.approx(speed*.78)
    assert message.header.stamp.sec==20


@pytest.mark.parametrize('speed',[-5.5,-2.36,2.36,5.5])
def test_large_encoder_spike_is_rejected_without_clipping(speed):
    original=reading(speed);message=correct(original,.78)
    assert not plausible(message,.75)
    assert message.twist.twist.linear.x==pytest.approx(speed*.78)
    assert original.twist.twist.linear.x==speed


def test_disabled_guard_and_lateral_nonfinite_validation():
    assert plausible(correct(reading(6.),.78),0.)
    assert not plausible(correct(reading(.4,.8),.78),.75)
    for invalid in [-1.,math.inf,math.nan]:
        with pytest.raises(ValueError):plausible(correct(reading(.3),.78),invalid)
