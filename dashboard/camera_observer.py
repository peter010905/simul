"""Read-only ROS camera observer. Never publishes commands or reads scenarios."""
import argparse
import json
import os
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--domain', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('osp_dashboard_camera')
    bridge = CvBridge()
    last_write = 0.0
    frames = 0

    def receive(message):
        nonlocal last_write, frames
        now = time.monotonic()
        if now - last_write < 1.0:
            return
        try:
            frame = bridge.imgmsg_to_cv2(message, 'bgr8')
            if frame.shape[1] > 960:
                frame = cv2.resize(frame, (960, round(frame.shape[0] * 960 / frame.shape[1])))
            ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
            if not ok:
                return
            (args.output / 'frame.tmp').write_bytes(encoded.tobytes())
            (args.output / 'frame.tmp').replace(args.output / 'frame.jpg')
            frames += 1
            metadata = {'received_unix': time.time(), 'source_sim_seconds':
                        message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
                        'frames': frames, 'domain': args.domain,
                        'width': frame.shape[1], 'height': frame.shape[0]}
            (args.output / 'metadata.tmp').write_text(json.dumps(metadata))
            (args.output / 'metadata.tmp').replace(args.output / 'metadata.json')
            last_write = now
        except Exception as exc:
            node.get_logger().warning(str(exc))

    node.create_subscription(Image, '/oakd/rgb/preview/image_raw', receive,
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
