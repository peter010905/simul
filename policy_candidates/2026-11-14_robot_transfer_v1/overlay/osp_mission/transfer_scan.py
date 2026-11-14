"""Gazebo scan relay with bounded, timestamp-preserving synthetic latency."""
from collections import deque
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from osp_mission.transfer_noise import SensorStress, environment_profile


class TransferScan(Node):
    def __init__(self):
        super().__init__('transfer_scan')
        if not self.get_parameter('use_sim_time').value:
            raise RuntimeError('Synthetic stress relay is for simulation only')
        name, seed = environment_profile()
        self.stress = SensorStress(name, seed)
        self.queue = deque(maxlen=32)
        self.output = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.status = self.create_publisher(String, 'mission/transfer_status', 10)
        self.create_subscription(LaserScan, 'scan_raw', self.receive, qos_profile_sensor_data)
        self.create_timer(.01, self.flush)
        self.create_timer(2., self.report)

    def receive(self, message):
        value = self.stress.scan(message)
        if value is not None:
            self.queue.append((self.get_clock().now().nanoseconds/1e9+self.stress.profile['delay'], value))

    def flush(self):
        now = self.get_clock().now().nanoseconds/1e9
        while self.queue and self.queue[0][0] <= now:
            self.output.publish(self.queue.popleft()[1])

    def report(self):
        self.status.publish(String(data=json.dumps(dict(profile=self.stress.name,
            parameters=self.stress.profile, received=self.stress.received,
            dropped_frames=self.stress.dropped, acquisition_stamp_preserved=True,
            ground_truth_input=False))))


def main(args=None):
    rclpy.init(args=args)
    node = TransferScan()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
