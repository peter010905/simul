"""Live TurtleBot4 board, with opt-in physical start/stop controls."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import signal
import sys
import threading
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'scripts'))
from r32_board_status import normalize as normalize_r32


def topic(namespace, name):
    if name.startswith('/'):
        return '/' + name.strip('/')
    return '/' + '/'.join(part for part in (namespace.strip('/'), name.strip('/')) if part)


def make_handler(monitor, control=None):
    from robot_board_control import START_WORKSPACES, valid_request
    routes = {"/api/control/" + action for action in (*START_WORKSPACES, "stop")}
    class Handler(BaseHTTPRequestHandler):
        def send_json(self, status, value):
            content = json.dumps(value).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        def do_POST(self):
            if self.path not in routes:
                self.send_json(404, {'error': 'Unknown control action'})
                return
            if control is None or not valid_request(self.headers, control.token):
                self.send_json(403, {'error': 'Control is disabled or request is not from this board'})
                return
            if self.headers.get('Content-Length', '0') != '2' or self.rfile.read(2) != b'{}':
                self.send_json(400, {'error': 'Expected an empty JSON object'})
                return
            try:
                result = control.request(self.path.rsplit('/', 1)[-1])
                self.send_json(202, result)
            except RuntimeError as exc:
                self.send_json(409, {'error': str(exc)})
            except OSError as exc:
                self.send_json(500, {'error': str(exc)})

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path in ('/', '/index.html'):
                content, mime = (HERE/'robot_web/index.html').read_bytes(), 'text/html; charset=utf-8'
            elif path == '/app.js':
                content, mime = (HERE/'robot_web/app.js').read_bytes(), 'text/javascript; charset=utf-8'
            elif path == '/style.css':
                content, mime = (HERE/'robot_web/style.css').read_bytes(), 'text/css; charset=utf-8'
            elif path == '/api/state':
                content, mime = json.dumps(monitor.snapshot()).encode(), 'application/json; charset=utf-8'
            elif path in ('/api/map.png', '/api/camera.jpg', '/api/scan.png'):
                with monitor.lock:
                    content = monitor.map_png if path == '/api/map.png' else monitor.scan_png if path == '/api/scan.png' else monitor.camera_jpeg
                if content is None:
                    self.send_error(503, 'ROS frame unavailable')
                    return
                mime = 'image/png' if path.endswith('.png') else 'image/jpeg'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        def log_message(self, format, *values):
            return

    return Handler


def run(args):
    import cv2
    import numpy as np
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from cv_bridge import CvBridge
    from nav2_msgs.msg import CollisionMonitorState
    from nav_msgs.msg import OccupancyGrid, Path as NavPath
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import Image, LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    from visualization_msgs.msg import MarkerArray
    from map_observer import render_map
    from robot_board_core import map_cell, scan_summary, mission_display
    from robot_board_control import BoardControl
    control = (BoardControl(HERE.parent, args.control_workspace, args.domain,
                            args.control_ntp_server, args.control_duration)
               if args.enable_control else None)

    class Monitor(Node):
        def __init__(self):
            super().__init__('osp_robot_board', parameter_overrides=[Parameter('use_sim_time', value=args.sim)])
            self.lock = threading.RLock()
            self.bridge = CvBridge()
            self.tf = Buffer(node=self)
            self.listener = TransformListener(self.tf, self)
            self.is_docked = None
            self.grid = None
            self.raw_scan = None
            self.filtered_scan = None
            self.status = {}
            self.collision = None
            self.plans = {'global': {}, 'local': {}}
            self.plan_counts = {'global': 0, 'local': 0}
            self.targets = []
            self.discovered_ids = set()
            self.latest_discovery = None
            self.timestamps = {}
            self.map_png = None
            self.camera_jpeg = None
            self.scan_png = None
            try:
                self.policy_prefix = get_package_prefix('osp_mission')
            except LookupError:
                self.policy_prefix = None
            base = args.namespace
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            if not args.sim:
                from irobot_create_msgs.msg import DockStatus
                self.create_subscription(DockStatus, topic(base, 'dock_status'), self.on_dock, qos_profile_sensor_data)
                self.create_subscription(String, topic(base, 'osp/status'), lambda msg: self.on_status(msg, native=True), latched)
            self.create_subscription(OccupancyGrid, topic(base, 'map'), self.on_map, latched)
            self.create_subscription(LaserScan, topic(base, args.scan), self.on_raw, qos_profile_sensor_data)
            self.create_subscription(LaserScan, topic(base, args.filtered_scan), self.on_filtered, qos_profile_sensor_data)
            self.create_subscription(String, topic(base, 'mission/status'), self.on_status, 10)
            self.create_subscription(CollisionMonitorState, topic(base, 'collision_monitor_state'), self.on_collision, 10)
            self.create_subscription(Image, topic(base, args.rgb), self.on_camera, qos_profile_sensor_data)
            self.create_subscription(MarkerArray, topic(base, 'targets'), self.on_targets, 10)
            self.create_subscription(NavPath, topic(base, 'plan'), lambda m: self.on_plan(m, 'global'), 10)
            self.create_subscription(NavPath, topic(base, 'local_plan'), lambda m: self.on_plan(m, 'local'), 10)
            self.create_timer(1.0, self.render)

        def on_dock(self, message):
            with self.lock:
                self.is_docked = bool(message.is_docked)
                self.mark('dock')

        def mark(self, name):
            self.timestamps[name] = time.time()

        def on_map(self, message):
            with self.lock:
                source = message.header.stamp.sec + message.header.stamp.nanosec/1e9
                previous = getattr(self, 'last_map_source_s', None)
                if args.sim and previous is not None and source < previous-1:
                    # A new Gazebo run reset /clock. Drop the previous run's
                    # status, paths and images instead of showing them as live.
                    self.status = {}
                    self.collision = None
                    self.plans = {'global': {}, 'local': {}}
                    self.plan_counts = {'global': 0, 'local': 0}
                    self.targets = []
                    self.discovered_ids.clear()
                    self.latest_discovery = None
                    self.raw_scan = None
                    self.filtered_scan = None
                    self.map_png = None
                    self.camera_jpeg = None
                    self.scan_png = None
                    self.timestamps.clear()
                self.last_map_source_s = source
                self.grid = message
                self.mark('map')

        def on_raw(self, message):
            with self.lock:
                self.raw_scan = message
                self.mark('raw_scan')

        def on_filtered(self, message):
            with self.lock:
                self.filtered_scan = message
                self.mark('filtered_scan')

        def on_status(self, message, native=False):
            try:
                parsed = json.loads(message.data)
                if not isinstance(parsed, dict):
                    return
            except ValueError:
                return
            with self.lock:
                parsed = normalize_r32(parsed) if native else parsed
                if (self.status.get('runtime') != parsed.get('runtime')
                        or (parsed.get('state') == 'idle' and self.status.get('state') != 'idle')
                        or (native and self.status.get('run_id') != parsed.get('run_id'))):
                    self.targets = []
                    self.discovered_ids.clear()
                    self.latest_discovery = None
                self.status = parsed
                if native:
                    self.update_targets(parsed['confirmed_targets'])
                self.mark('mission_status')

        def on_collision(self, message):
            with self.lock:
                self.collision = {'action_type': int(message.action_type),
                                  'action': {0: 'none', 1: 'STOP', 2: 'SLOWDOWN',
                                             3: 'APPROACH', 4: 'LIMIT'}.get(message.action_type, 'unknown'),
                                  'polygon': message.polygon_name}
                self.mark('collision_monitor')

        def on_targets(self, message):
            with self.lock:
                if self.status.get('runtime') != 'upstream-r32':
                    self.update_targets([(int(m.id), m.pose.position.x, m.pose.position.y)
                                         for m in message.markers if m.ns == 'point' and m.action == 0
                                         and m.header.frame_id == args.map_frame])

        def update_targets(self, targets):
            self.targets = targets
            new_ids = sorted({ident for ident, _, _ in targets} - self.discovered_ids)
            if new_ids:
                self.discovered_ids.update(new_ids)
                self.latest_discovery = {'ids': new_ids, 'time': time.time()}
            self.mark('targets')

        def on_plan(self, message, kind):
            if message.header.frame_id != args.map_frame:
                return
            points = [(p.pose.position.x, p.pose.position.y) for p in message.poses]
            with self.lock:
                self.plan_counts[kind] += 1
                self.plans[kind] = {'sequence': self.plan_counts[kind], 'points': points}
                self.mark(kind + '_plan')

        def on_camera(self, message):
            if time.time() - self.timestamps.get('camera', 0) < .25:
                return
            try:
                frame = self.bridge.imgmsg_to_cv2(message, 'bgr8')
                if frame.shape[1] > 960:
                    frame = cv2.resize(frame, (960, round(frame.shape[0]*960/frame.shape[1])))
                ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 78])
                if ok:
                    with self.lock:
                        self.camera_jpeg = jpg.tobytes()
                        self.mark('camera')
            except Exception as exc:
                self.get_logger().warning(f'Board camera unavailable: {exc}', throttle_duration_sec=10.)

        def pose(self):
            try:
                stamped = self.tf.lookup_transform(args.map_frame, args.base_frame, Time())
                stamp = stamped.header.stamp.sec + stamped.header.stamp.nanosec/1e9
                age = self.get_clock().now().nanoseconds/1e9 - stamp
                with self.lock:
                    self.pose_age_s = age
                if age > 3 or age < -.5:
                    return None
                tr = stamped.transform
                q = tr.rotation
                import math
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                return (tr.translation.x, tr.translation.y, yaw)
            except Exception:
                with self.lock:
                    self.pose_age_s = None
                return None

        def render(self):
            with self.lock:
                grid = self.grid
                plans = {k: dict(v) for k, v in self.plans.items()}
                targets = list(self.targets)
                raw_scan = self.raw_scan
                filtered_scan = self.filtered_scan
            if raw_scan is not None:
                self.render_scan(raw_scan, filtered_scan)
            if grid is None:
                return
            pose = self.pose()
            info = grid.info
            q = info.origin.orientation
            import math
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            origin = (info.origin.position.x, info.origin.position.y, yaw)
            cells = np.asarray(grid.data, dtype=np.int16).reshape(info.height, info.width)
            frame = render_map(cells, info.resolution, origin, plans, pose, targets=targets)
            ok, png = cv2.imencode('.png', frame)
            with self.lock:
                self.start_cell = map_cell(cells, info.resolution, origin, pose)
                self.robot_pose = pose
                if ok:
                    self.map_png = png.tobytes()
                    self.mark('map_image')

        def render_scan(self, raw, filtered):
            import math
            try:
                tr = self.tf.lookup_transform(args.base_frame, raw.header.frame_id, Time()).transform
                q = tr.rotation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                canvas = np.full((640,640,3), (25,35,48), np.uint8)
                center = (320,320)
                scale = 450
                cv2.circle(canvas, center, round(.189*scale), (75,90,110), 2)
                cv2.line(canvas, (320,320), (320+round(.25*scale),320), (80,80,100), 1)
                matched = filtered is not None and raw.header.stamp == filtered.header.stamp and len(raw.ranges)==len(filtered.ranges)
                for index, distance in enumerate(raw.ranges):
                    if not (math.isfinite(distance) and raw.range_min < distance < min(raw.range_max,.65)):
                        continue
                    angle = yaw+raw.angle_min+index*raw.angle_increment
                    x = tr.translation.x+distance*math.cos(angle)
                    y = tr.translation.y+distance*math.sin(angle)
                    px,py = 320+round(x*scale),320-round(y*scale)
                    if not (0 <= px < 640 and 0 <= py < 640):
                        continue
                    removed = matched and not math.isfinite(filtered.ranges[index])
                    cv2.circle(canvas,(px,py),2,(65,85,255) if removed else (230,220,75),-1)
                sx = 320+round(tr.translation.x*scale)
                sy = 320-round(tr.translation.y*scale)
                cv2.circle(canvas,(sx,sy),5,(255,255,255),-1)
                cv2.circle(canvas,center,5,(110,235,140),-1)
                cv2.putText(canvas,'BASE_LINK  +X RIGHT  |  BODY REFERENCE r=0.189m',
                            (20,30),cv2.FONT_HERSHEY_SIMPLEX,.51,(225,235,235),1,cv2.LINE_AA)
                ok,png = cv2.imencode('.png',canvas)
                if ok:
                    with self.lock:
                        self.scan_png = png.tobytes()
                        self.mark('scan_image')
            except Exception as exc:
                self.get_logger().warning(f'Board scan view unavailable: {exc}', throttle_duration_sec=10.)

        def snapshot(self):
            with self.lock:
                stamps = dict(self.timestamps)
                status = dict(self.status)
                scan = scan_summary(self.raw_scan, self.filtered_scan)
                collision = self.collision
                cell = getattr(self, 'start_cell', {'reason': 'map_or_pose_unavailable'})
                pose = getattr(self, 'robot_pose', None)
                pose_age = getattr(self, 'pose_age_s', None)
                plans = dict(self.plan_counts)
                camera_ready = self.camera_jpeg is not None
                map_ready = self.map_png is not None
                scan_ready = self.scan_png is not None
                discovered_ids = sorted(self.discovered_ids)
                latest_discovery = self.latest_discovery
                docked = self.is_docked
            now = time.time()
            ages = {name: round(now-stamp, 1) for name, stamp in stamps.items()}
            return {'server_time': now, 'mode': 'simulation' if args.sim else 'hardware',
                    'namespace': args.namespace, 'ros_domain_id': args.domain,
                    'policy_prefix': self.policy_prefix, 'status': status,
                    'mission_display': mission_display(status), 'collision_monitor': collision,
                    'start_cell': cell, 'robot_pose': pose, 'pose_age_s': pose_age,
                    'scan': scan, 'ages_s': ages,
                    'docked': docked if ages.get('dock', float('inf')) <= 2. else None,
                    'discovered_ids': discovered_ids,
                    'latest_discovery': latest_discovery,
                    'plans': plans, 'camera_ready': camera_ready, 'map_ready': map_ready,
                    'scan_ready': scan_ready,
                    'read_only': control is None,
                    'control': control.snapshot() if control else {'enabled': False}}

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    monitor = Monitor()
    worker = threading.Thread(target=rclpy.spin, args=(monitor,), daemon=True)
    worker.start()

    server = ThreadingHTTPServer((args.host, args.port), make_handler(monitor, control))
    signal.signal(signal.SIGTERM, lambda *_: threading.Thread(target=server.shutdown, daemon=True).start())
    print(f'{"Simulation" if args.sim else "Physical robot"} board: http://{args.host}:{args.port}  ROS_DOMAIN_ID={args.domain}', flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        rclpy.shutdown()
        worker.join(timeout=2)
        monitor.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--domain', type=int, default=0)
    parser.add_argument('--namespace', default='')
    parser.add_argument('--scan', default='scan')
    parser.add_argument('--filtered-scan', default='scan_self_filtered')
    parser.add_argument('--rgb', default='oakd/rgb/preview/image_raw')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--sim', action='store_true', help='Use ROS /clock for TF freshness')
    parser.add_argument('--enable-control', action='store_true', help='Enable explicit physical start/stop buttons')
    parser.add_argument('--control-workspace', type=Path, default=Path.home()/'face_intent_flow_v4r2_ws')
    parser.add_argument('--control-ntp-server', default='192.168.185.3')
    parser.add_argument('--control-duration', type=float, default=600.)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232 or not 1024 <= args.port <= 65535:
        parser.error('Domain must be 0..232 and port 1024..65535')
    if args.enable_control and (args.sim or args.namespace or args.scan != 'scan' or args.rgb != 'oakd/rgb/preview/image_raw'):
        parser.error('Control currently supports the root-namespace physical hardware configuration only')
    import math
    if not math.isfinite(args.control_duration) or args.control_duration <= 0:
        parser.error('Control duration must be positive and finite')
    import os
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    run(args)


if __name__ == '__main__':
    main()
