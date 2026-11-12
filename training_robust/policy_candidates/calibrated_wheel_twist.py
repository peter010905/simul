"""Apply a fixed encoder velocity calibration;no world-pose or scenario inputs."""
import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry


def correct(message,scale):
    if not math.isfinite(scale) or scale<=0:raise ValueError('Invalid fixed encoder scale')
    if not message.child_frame_id:raise ValueError('Encoder twist needs its body frame')
    twist=message.twist.twist
    values=[twist.linear.x,twist.linear.y,twist.linear.z,twist.angular.x,twist.angular.y,twist.angular.z]
    if not all(math.isfinite(value) for value in [*values,*message.twist.covariance]):
        raise ValueError('Nonfinite encoder twist')
    result=TwistWithCovarianceStamped();result.header=copy.deepcopy(message.header)
    result.header.frame_id=message.child_frame_id;result.twist=copy.deepcopy(message.twist)
    result.twist.twist.linear.x*=scale
    # Transform cross-covariances;retain the original forward variance as a floor.
    covariance=list(result.twist.covariance);original=covariance[0]
    for index in range(6):
        covariance[index]*=scale;covariance[index*6]*=scale
    covariance[0]=max(covariance[0],original);result.twist.covariance=covariance
    return result


class CalibratedWheelTwist(Node):
    def __init__(self):
        super().__init__('calibrated_wheel_twist');self.declare_parameter('linear_scale',1.)
        self.scale=float(self.get_parameter('linear_scale').value)
        if not math.isfinite(self.scale) or self.scale<=0:raise ValueError('Invalid linear_scale')
        self.output=self.create_publisher(TwistWithCovarianceStamped,'wheel/twist_calibrated',10)
        self.create_subscription(Odometry,'wheel/odom',self.receive,qos_profile_sensor_data)

    def receive(self,message):
        try:self.output.publish(correct(message,self.scale))
        except ValueError as exc:self.get_logger().warning(str(exc),throttle_duration_sec=5.)


def main(args=None):
    rclpy.init(args=args);node=CalibratedWheelTwist()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
