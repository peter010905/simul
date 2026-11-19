"""Relay a simulation's reported mission state to the upstream sensor board."""
import json
import math
from pathlib import Path
import sys

import rclpy
from nav_msgs.msg import Path as NavPath
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


def main():
    root = Path(sys.argv[1])
    track = root.name == 'track-recordings'
    duration = 600 if track else 120
    rclpy.init()
    node = rclpy.create_node('flow_dashboard_telemetry')
    pub = node.create_publisher(String, '/osp/status', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    points = {}
    goal = None

    def markers(msg):
        points.clear()
        for marker in msg.markers:
            if marker.ns == 'point' and marker.action == 0:
                points[marker.id] = (marker.pose.position.x, marker.pose.position.y)

    def plan(msg):
        nonlocal goal
        if msg.poses and msg.header.frame_id == 'map':
            end = msg.poses[-1].pose
            goal = dict(x=end.position.x, y=end.position.y,
                        yaw=2*math.atan2(end.orientation.z, end.orientation.w), kind='Nav2 경로 끝점')

    def status(msg):
        nonlocal goal
        state = json.loads(msg.data)
        elapsed = state.get('elapsed_seconds') or 0
        action = state.get('active_action')
        phase = {'idle':'READY', 'saving':'FINISHING', 'done':'FINISHED'}.get(state.get('state'))
        if phase == 'READY':
            points.clear()
            goal = None
        if phase is None:
            phase = {'nav':'NAVIGATE', 'spin':'SCAN'}.get(action, 'EXPLORE')
        pattern = '*-track-*' if track else '*-[123]'
        runs = [d for d in root.glob(pattern) if (d/'launch.log').is_file()]
        active = [d for d in runs if not (d/'metrics.json').is_file()]
        current = max(active or runs, key=lambda d:(d/'launch.log').stat().st_mtime, default=None)
        targets = [dict(id=i, x=x, y=y, confirmed=True, approached=i in state.get('approached_ids', []))
                   for i,(x,y) in points.items()]
        payload = dict(state=phase, elapsed=elapsed, remaining=max(0, duration-elapsed),
                       run_id=current.name if current else '', reason=state.get('termination_reason',''),
                       readiness=state.get('readiness_reason'), goal=goal if action=='nav' else None, targets=targets)
        pub.publish(String(data=json.dumps(payload)))

    node.create_subscription(String, '/mission/status', status, 10)
    node.create_subscription(MarkerArray, '/targets', markers, 10)
    node.create_subscription(NavPath, '/plan', plan, 10)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
