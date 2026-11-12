import pytest
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from tf2_ros import Buffer, ExtrapolationException


def transform(parent, child, stamp, x):
    message = TransformStamped()
    message.header.frame_id = parent
    message.child_frame_id = child
    message.header.stamp = Time(seconds=stamp).to_msg()
    message.transform.rotation.w = 1.
    message.transform.translation.x = x
    return message


def test_current_odom_pose_requires_map_transform_time_coverage():
    buffer = Buffer()
    buffer.set_transform(transform('map', 'odom', 10.2, .1), 'slam')
    buffer.set_transform(transform('map', 'odom', 10.3, .1), 'slam')
    buffer.set_transform(transform('odom', 'base_link', 10.304, 1.), 'odometry')
    with pytest.raises(ExtrapolationException):
        buffer.lookup_transform('map', 'base_link', Time(seconds=10.304))
    # SLAM's supported restamp_tf option publishes the same correction using
    # now()+transform_timeout, covering the contemporary odometry timestamp.
    buffer.set_transform(transform('map', 'odom', 10.504, .1), 'slam')
    result = buffer.lookup_transform('map', 'base_link', Time(seconds=10.304))
    assert result.transform.translation.x == pytest.approx(1.1)
