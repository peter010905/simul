"""Model only held-out verified sensor-frame returns upstream of the physical self-filter."""

import json
import math
import os
from pathlib import Path


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    if (
        os.environ.get("ROS_DOMAIN_ID") not in ("174", "175")
        or os.environ.get("ROS_LOCALHOST_ONLY") != "1"
    ):
        raise RuntimeError("Isolated Gazebo only")
    profile = json.loads(
        Path(os.environ["OSP_REAL_SIM_PROFILE"])
        .with_name("scan_signature.json")
        .read_text()
    )
    beams, angle_min, increment, _ = profile["geometry"]
    corrected = {
        **{int(i): v for i, v in profile["short"].items()},
        **{int(i): math.inf for i in profile["missing"]},
    }
    rclpy.init()
    node = rclpy.create_node("twin_raw_lidar")
    publisher = node.create_publisher(LaserScan, "scan", qos_profile_sensor_data)

    def send(msg):
        if (
            len(msg.ranges) != beams
            or abs(msg.angle_min - angle_min) > 0.001
            or abs(msg.angle_increment - increment) > 0.0001
        ):
            raise ValueError("Unexpected Gazebo scan geometry")
        for i, value in corrected.items():
            msg.ranges[i] = value
        publisher.publish(msg)

    node.create_subscription(
        LaserScan, "twin/render_scan", send, qos_profile_sensor_data
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
