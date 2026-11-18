import math

import pytest
from nav_msgs.msg import Odometry

from osp_mission.calibrated_wheel_twist import correct


def message():
    value=Odometry();value.header.frame_id='odom';value.child_frame_id='base_link'
    value.header.stamp.sec=123;value.header.stamp.nanosec=456
    value.twist.twist.linear.x=.4;value.twist.twist.angular.z=.2
    value.twist.covariance[0]=.001;value.twist.covariance[35]=.01
    value.twist.covariance[5]=value.twist.covariance[30]=.0001
    return value


def test_only_encoder_forward_velocity_changes_with_original_stamp_and_body_frame():
    raw=message();result=correct(raw,.78)
    assert result.header.stamp==raw.header.stamp and result.header.frame_id=='base_link'
    assert result.twist.twist.linear.x==pytest.approx(.312)
    assert result.twist.twist.angular.z==.2 and raw.twist.twist.linear.x==.4


def test_covariance_cross_terms_transform_without_increasing_forward_confidence():
    result=correct(message(),.78)
    assert result.twist.covariance[0]==.001
    assert result.twist.covariance[35]==.01
    assert result.twist.covariance[5]==pytest.approx(.000078)
    assert result.twist.covariance[30]==pytest.approx(.000078)


def test_invalid_scale_never_produces_a_twist():
    for scale in [0.,-1.,math.nan]:
        with pytest.raises(ValueError):correct(message(),scale)


def test_missing_body_frame_and_nonfinite_velocity_are_rejected():
    raw=message();raw.child_frame_id=''
    with pytest.raises(ValueError):correct(raw,.78)
    raw=message();raw.twist.twist.linear.x=math.nan
    with pytest.raises(ValueError):correct(raw,.78)
