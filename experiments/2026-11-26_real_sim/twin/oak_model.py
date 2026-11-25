"""Render pinhole geometry in Gazebo; publish the original OAK preview calibration separately."""

import json
import math
import os
from pathlib import Path


def configure(doc, p):
    def child(parent, tag, text=None):
        n = next((n for n in parent.childNodes if n.nodeName == tag), None)
        if n is None:
            n = doc.createElement(tag)
            parent.appendChild(n)
        if text is not None:
            while n.firstChild:
                n.removeChild(n.firstChild)
            n.appendChild(doc.createTextNode(str(text)))
        return n

    sensors = [
        s
        for s in doc.getElementsByTagName("sensor")
        if s.getAttribute("type") == "rgbd_camera"
    ]
    if len(sensors) != 1:
        raise ValueError("Expected one RGB-D camera")
    camera = child(sensors[0], "camera")
    intr = child(child(camera, "lens"), "intrinsics")
    for key, val in dict(
        fx=p["render_fx"], fy=p["render_fy"], cx=p["k"][2], cy=p["k"][5], s=0
    ).items():
        child(intr, key, val)
    child(camera, "horizontal_fov", 2 * math.atan(p["width"] / (2 * p["render_fx"])))


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo

    if (
        os.environ.get("ROS_DOMAIN_ID") not in ("174", "175")
        or os.environ.get("ROS_LOCALHOST_ONLY") != "1"
    ):
        raise RuntimeError("Sim-only preview metadata")
    p = json.loads(
        Path(os.environ["OSP_REAL_SIM_PROFILE"]).with_name("optics.json").read_text()
    )
    rclpy.init()
    node = rclpy.create_node("twin_preview_camera_info")
    pub = node.create_publisher(
        CameraInfo, "oakd/rgb/preview/camera_info", qos_profile_sensor_data
    )

    def publish(source):
        if (source.width, source.height) != (p["width"], p["height"]):
            return
        msg = CameraInfo()
        msg.header = source.header
        # Use the optical frame from the simulator TF tree.
        for key in ("width", "height", "k", "r", "p", "d", "distortion_model"):
            setattr(msg, key, p[key])
        pub.publish(msg)

    node.create_subscription(
        CameraInfo, "twin/render_camera_info", publish, qos_profile_sensor_data
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
