"""Publish the Gazebo camera and relay bounded teleoperation inside domain 86."""

import hmac
import json
import math
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Lock, Thread
from time import monotonic

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class SimBridge(Node):
    def __init__(self):
        super().__init__("marker_board_bridge", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True),
        ])
        self.lock = Lock()
        self.frame = None
        self.sequence = 0
        self.odom = None
        self.command = (0.0, 0.0)
        self.expires = 0
        self.last_capture = 0
        self.converter = CvBridge()
        self.velocity = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.image,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.pose, qos_profile_sensor_data)
        # Wall-clock timer keeps the control lease independent of Gazebo's speed.
        self.create_timer(.05, self.tick, clock=rclpy.clock.Clock(
            clock_type=rclpy.clock.ClockType.STEADY_TIME))

    def image(self, message):
        now = monotonic()
        if now - self.last_capture < .12:
            return
        self.last_capture = now
        image = self.converter.imgmsg_to_cv2(message, "bgr8")
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 89])
        if ok:
            with self.lock:
                self.sequence += 1
                stamp = message.header.stamp
                self.frame = (self.sequence, now, stamp.sec + stamp.nanosec / 1e9,
                              encoded.tobytes())

    def pose(self, message):
        pose = message.pose.pose
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.odom = [pose.position.x, pose.position.y, math.degrees(yaw)]

    def tick(self):
        with self.lock:
            active = monotonic() < self.expires
            linear, angular = self.command if active else (0.0, 0.0)
        stamp = self.get_clock().now().to_msg()
        if stamp.sec == 0 and stamp.nanosec == 0:
            return
        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.velocity.publish(msg)


def api(node, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, status, body=b"", content="application/json", **headers):
            self.send_response(status)
            self.send_header("Content-Type", content)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for key, value in headers.items():
                self.send_header(key.replace("_", "-"), str(value))
            self.end_headers()
            self.wfile.write(body)

        def allowed(self):
            return hmac.compare_digest(self.headers.get("X-Sim-Token", ""), token)

        def do_GET(self):
            if not self.allowed():
                return self.send(403)
            with node.lock:
                if self.path == "/state":
                    payload = dict(odom=node.odom, camera=node.sequence)
                    return self.send(200, json.dumps(payload).encode())
                frame = node.frame
            if self.path != "/frame":
                return self.send(404)
            if frame is None or monotonic() - frame[1] > 2:
                return self.send(204)
            seq, _, stamp, data = frame
            return self.send(200, data, "image/jpeg", X_Frame=seq, X_Sim_Time=stamp)

        def do_POST(self):
            if not self.allowed() or self.path != "/drive":
                return self.send(403)
            try:
                length = int(self.headers.get("Content-Length", 0))
                if not 1 <= length <= 256:
                    raise ValueError
                data = json.loads(self.rfile.read(length))
                linear, angular = float(data["linear"]), float(data["angular"])
                if not math.isfinite(linear) or not math.isfinite(angular):
                    raise ValueError
                if abs(linear) > .24 or abs(angular) > 1.0:
                    raise ValueError
            except (ValueError, TypeError, KeyError):
                return self.send(400)
            with node.lock:
                ready = node.odom is not None and node.frame is not None and monotonic() - node.frame[1] <= 2
                if not ready and (linear or angular):
                    return self.send(503)
                node.command = (linear, angular)
                node.expires = monotonic() + .30
            return self.send(204)

    return Handler


def main():
    if (os.environ.get("ROS_DOMAIN_ID") != "86"
            or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST"
            or not os.environ.get("GZ_PARTITION", "").startswith("osp_marker_board_")):
        raise SystemExit("Marker board requires the isolated simulation session")
    token = Path(__file__).with_name("bridge-token").read_text().strip()
    if len(token) < 32:
        raise SystemExit("Missing bridge token")
    cv2.setNumThreads(1)
    rclpy.init()
    node = SimBridge()
    server = ThreadingHTTPServer(("192.168.252.2", 8793), api(node, token))
    Thread(target=server.serve_forever, daemon=True).start()
    print("Camera and control bridge waiting for Gazebo frames", flush=True)
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        server.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
