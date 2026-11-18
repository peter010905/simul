"""Reject only returns inside the configured base footprint and an 18 cm disk."""
import copy
import json
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

FOOTPRINT = ((.189, 0.), (.134, -.134), (0., -.189), (-.134, -.134),
             (-.189, 0.), (-.134, .134), (0., .189), (.134, .134))
INSET_M = .005
MAX_RADIUS_M = .18


def inside_body(x, y):
    if not math.isfinite(x+y) or x*x+y*y >= MAX_RADIUS_M**2:
        return False
    for a, b in zip(FOOTPRINT, FOOTPRINT[1:]+FOOTPRINT[:1]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        if dx*(y-a[1])-dy*(x-a[0]) >= -INSET_M*math.hypot(dx, dy):
            return False
    return True


def filter_ranges(scan, sensor_x, sensor_y, sensor_yaw):
    if not all(math.isfinite(v) for v in (sensor_x, sensor_y, sensor_yaw,
                                         scan.angle_min, scan.angle_increment)):
        raise ValueError('Invalid scan transform or angular metadata')
    values = list(scan.ranges)
    removed = 0
    for i, distance in enumerate(values):
        if not (math.isfinite(distance) and scan.range_min < distance < scan.range_max):
            continue
        angle = sensor_yaw+scan.angle_min+i*scan.angle_increment
        if inside_body(sensor_x+distance*math.cos(angle), sensor_y+distance*math.sin(angle)):
            values[i] = math.inf
            removed += 1
    return values, removed


def route_costmap_scans(config, raw, filtered):
    result = copy.deepcopy(config)
    count = 0
    for key in ('local_costmap', 'global_costmap'):
        parameters = result[key][key]['ros__parameters']
        if tuple(tuple(p) for p in json.loads(parameters['footprint'])) != FOOTPRINT:
            raise ValueError('Self scan mask must match the configured base footprint')
        for layer in parameters.values():
            if isinstance(layer, dict) and isinstance(layer.get('scan'), dict):
                source = layer['scan']
                if str(source.get('topic', '')).lstrip('/') == raw.lstrip('/'):
                    source['topic'] = filtered
                    count += 1
    if count < 2:
        raise ValueError('Both costmaps must consume the filtered scan')
    # The independent collision monitor continues to consume the raw scan.
    if result['collision_monitor'] != config['collision_monitor']:
        raise ValueError('Raw collision monitor configuration changed')
    return result


class BodyScanFilter(Node):
    def __init__(self):
        super().__init__('body_scan_filter')
        for name, value in {'input_topic':'scan', 'output_topic':'scan_self_filtered',
                            'base_frame':'base_link'}.items():
            self.declare_parameter(name, value)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.output = self.create_publisher(LaserScan, self.get_parameter('output_topic').value,
                                            qos_profile_sensor_data)
        self.diagnostics = self.create_publisher(String, 'mission/self_scan_status', 10)
        self.total_removed = self.total_scans = self.tf_waits = 0
        self.create_subscription(LaserScan, self.get_parameter('input_topic').value,
                                 self.on_scan, qos_profile_sensor_data)
        self.create_timer(1., self.report)

    def on_scan(self, scan):
        try:
            t = self.tf.lookup_transform(self.get_parameter('base_frame').value,
                    scan.header.frame_id, Time.from_msg(scan.header.stamp),
                    timeout=Duration(seconds=.02)).transform
            q = t.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            values, removed = filter_ranges(scan, t.translation.x, t.translation.y, yaw)
        except Exception as exc:
            # Raw scan is still published by the robot and used by its safety
            # monitor. Withhold planning data until exact-time geometry exists.
            self.tf_waits += 1
            self.get_logger().warning(f'Filtered scan waiting for TF: {exc}', throttle_duration_sec=5.)
            return
        result = copy.deepcopy(scan)
        result.ranges = values
        self.total_removed += removed
        self.total_scans += 1
        self.output.publish(result)

    def report(self):
        self.diagnostics.publish(String(data=json.dumps(dict(scans=self.total_scans,
            removed=self.total_removed, tf_waits=self.tf_waits,
            maximum_body_radius_m=MAX_RADIUS_M, footprint_inset_m=INSET_M,
            raw_collision_monitor=True))))


def main(args=None):
    rclpy.init(args=args)
    node = BodyScanFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
