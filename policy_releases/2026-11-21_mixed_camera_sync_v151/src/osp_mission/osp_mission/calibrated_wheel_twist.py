"""Apply a fixed encoder velocity calibration;no world-pose or scenario inputs."""
import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import json
from osp_mission.encoder_delta_twist import EncoderDelta


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


def plausible(message, maximum):
    """Zero disables the guard; a rejected sample is never clipped or fabricated."""
    if not math.isfinite(maximum) or maximum<0:
        raise ValueError('Invalid calibrated speed limit')
    speed=math.hypot(message.twist.twist.linear.x,message.twist.twist.linear.y)
    return math.isfinite(speed) and (maximum==0 or speed<=maximum)


class CalibratedWheelTwist(Node):
    def __init__(self):
        super().__init__('calibrated_wheel_twist');self.declare_parameter('linear_scale',1.)
        self.scale=float(self.get_parameter('linear_scale').value)
        if not math.isfinite(self.scale) or self.scale<=0:raise ValueError('Invalid linear_scale')
        self.declare_parameter('maximum_calibrated_linear_speed',0.)
        self.maximum=float(self.get_parameter('maximum_calibrated_linear_speed').value)
        if not math.isfinite(self.maximum) or self.maximum<0:raise ValueError('Invalid calibrated speed limit')
        self.declare_parameter('velocity_source','reported')
        self.velocity_source=self.get_parameter('velocity_source').value
        if self.velocity_source not in ('reported','pose_delta'):raise ValueError('Invalid encoder velocity source')
        self.delta=EncoderDelta()
        self.accepted=0;self.rejected=0;self.maximum_rejected_speed=0.
        self.status=self.create_publisher(String,'mission/wheel_twist_status',10)
        self.create_timer(5.,self.publish_status)
        self.output=self.create_publisher(TwistWithCovarianceStamped,'wheel/twist_calibrated',10)
        self.create_subscription(Odometry,'wheel/odom',self.receive,qos_profile_sensor_data)

    def receive(self,message):
        try:
            if self.velocity_source=='pose_delta':
                message=self.delta.update(message)
                if message is None:return
            calibrated=correct(message,self.scale)
            if not plausible(calibrated,self.maximum):
                self.rejected+=1
                self.maximum_rejected_speed=max(self.maximum_rejected_speed,
                    math.hypot(calibrated.twist.twist.linear.x,calibrated.twist.twist.linear.y))
                self.get_logger().warning('Rejected implausible encoder chassis speed before EKF fusion',throttle_duration_sec=5.)
                return
            self.accepted+=1;self.output.publish(calibrated)
        except ValueError as exc:self.get_logger().warning(str(exc),throttle_duration_sec=5.)


    def publish_status(self):
        self.status.publish(String(data=json.dumps(dict(
            velocity_source=self.velocity_source,linear_scale=self.scale,maximum_calibrated_linear_speed=self.maximum,
            accepted=self.accepted,rejected=self.rejected,
            maximum_rejected_speed_m_s=self.maximum_rejected_speed,
            ground_truth_input=False))))


def main(args=None):
    rclpy.init(args=args);node=CalibratedWheelTwist()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
