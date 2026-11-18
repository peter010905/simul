"""Start a prepared hardware runtime without rebuilding or manual start RPCs.

Only `start` grants motion. `check`/`status` are read-only. `stop` cancels
actions, ends the tracked launch, and verifies measured odometry. ROS imports
are lazy so the safety/start state machine can also be tested on the Mac.
"""
import argparse
from collections import deque
from dataclasses import dataclass
from r32_board_status import normalize as normalize_r32
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


CONFLICTS = {"mission", "aruco_detector", "slam_toolbox", "controller_server",
             "planner_server", "behavior_server", "bt_navigator",
             "collision_monitor", "velocity_smoother", "body_scan_filter"}
CANCEL_ACTIONS = ("navigate_to_pose", "backup", "spin", "drive_on_heading", "dock", "undock")
REQUIRED = ("scan", "camera", "camera_info", "odom", "hazards", "dock")


@dataclass
class StopFlag:
    stopped: bool = False


class Sensors:
    def __init__(self):
        self.samples = {}
        self.odom = deque(maxlen=200)

    def update(self, name, received, stamp=None, value=None):
        self.samples[name] = (received, stamp, value)
        if name == "odom":
            self.odom.append((received, *value))

    def reason(self, wall, ros_now):
        # The dock powers down lidar and RGB. Report a fresh positive dock or
        # hazard signal even when one of those streams has not produced a frame.
        dock = self.samples.get("dock")
        if dock and 0 <= wall - dock[0] <= 2. and dock[2]:
            return "robot_docked: move off the dock before start (no automatic undock)"
        hazards = self.samples.get("hazards")
        if hazards and 0 <= wall - hazards[0] <= 2.:
            dangerous = sorted(set(hazards[2]) & {1, 2, 3, 4})
            if dangerous:
                return "active_hazard_types:" + ",".join(map(str, dangerous))
        for name in REQUIRED:
            sample = self.samples.get(name)
            if sample is None:
                return "waiting_for_" + name
            received, stamp, _ = sample
            if wall - received > 2.0:
                return name + "_not_receiving"
            limit = 0.65 if name in ("camera", "camera_info") else 2.0
            if stamp is not None and not -0.1 <= ros_now - stamp <= limit:
                return name + "_clock_or_stamp_stale"
        return ""

    def stationary(self, wall):
        recent = [v for v in self.odom if wall - v[0] <= 1.0]
        return len(recent) >= 3 and all(
            all(math.isfinite(x) for x in v[1:]) and abs(v[1]) < 0.003 and abs(v[2]) < 0.01
            for v in recent)


class BumpRecoveryGuard:
    """Let the mission retreat after any number of contacts; bound held contact."""
    def __init__(self):
        self.last_contact = None
        self.active_since = None
        self.previous = False
        self.total = 0

    def check(self, now, types):
        if set(types) & {2, 3, 4}:
            return "Hardware cliff/stall/wheel-drop hazard"
        bump = 1 in types
        if bump:
            if not self.previous:
                self.active_since = now
                if self.last_contact is None or now - self.last_contact >= 2.:
                    self.last_contact = now
                    self.total += 1
            if self.active_since is not None and now - self.active_since >= 10.:
                return "Bumper held for 10 seconds: retreat did not clear contact"
        else:
            self.active_since = None
        self.previous = bump
        return ""


class StartGate:
    """One outstanding start request; a lost reply fails closed without retry."""
    def __init__(self, timeout=5.0):
        self.timeout = timeout
        self.future = None
        self.requested = None
        self.next_attempt = 0.0
        self.started = False

    def step(self, now, stopped, reason, state, client, request):
        if stopped:
            raise InterruptedError("operator_stop")
        if reason:
            if reason.startswith(("active_hazard", "robot_docked")) and self.future is not None:
                raise RuntimeError("Safety changed during start: " + reason)
            if self.future is None:
                return False
            # An outstanding RPC is answered by the mission's own fresh-data
            # readiness gate. Brief camera delay must not discard that answer.
        if self.future is not None:
            if not self.future.done():
                if now - self.requested > self.timeout:
                    raise TimeoutError("Start reply missing; not retrying an ambiguous start")
                return False
            response = self.future.result()
            self.future = None
            if response.success:
                self.started = True
                return True
            if "already started" in response.message.lower():
                raise RuntimeError(response.message)
            self.next_attempt = now + 0.25
        if state and state != "idle":
            raise RuntimeError("Refusing to start an existing mission: " + state)
        if state == "idle" and now >= self.next_attempt and client.service_is_ready():
            self.future = client.call_async(request())
            self.requested = now
        return False

    def cancel(self, client):
        if self.future is not None:
            self.future.cancel()
            client.remove_pending_request(self.future)
            self.future = None


class RobotIO:
    def __init__(self, domain, runtime="flow"):
        import rclpy
        from action_msgs.srv import CancelGoal
        from geometry_msgs.msg import TwistStamped
        from irobot_create_msgs.msg import DockStatus, HazardDetectionVector
        from nav_msgs.msg import Odometry
        from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import CameraInfo, Image, LaserScan
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        self.rclpy = rclpy
        rclpy.init(domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node("osp_robot_session_" + str(os.getpid()))
        self.sensors = Sensors()
        self.status = {}
        self.status_at = -math.inf
        self.TwistStamped, self.Trigger, self.CancelGoal = TwistStamped, Trigger, CancelGoal
        self.runtime = runtime
        prefix = "/osp" if runtime == "upstream-r32" else "/mission"
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) if runtime == "upstream-r32" else 10
        self.start_client = self.node.create_client(Trigger, prefix + "/start")
        self.stop_client = self.node.create_client(Trigger, prefix + "/stop")
        self.cancel_clients = {name: self.node.create_client(CancelGoal, "/" + name + "/_action/cancel_goal")
                               for name in CANCEL_ACTIONS}
        # Creating a publisher is deferred until an explicit stop or owned cleanup.
        self.zero_pub = None
        def stamped(name, msg, value=None):
            stamp = msg.header.stamp
            self.sensors.update(name, time.monotonic(), stamp.sec + stamp.nanosec / 1e9, value)
        self.subscriptions = [
            self.node.create_subscription(LaserScan, "/scan", lambda m: stamped("scan", m), qos_profile_sensor_data),
            self.node.create_subscription(Image, "/oakd/rgb/preview/image_raw", lambda m: stamped("camera", m), qos_profile_sensor_data),
            self.node.create_subscription(CameraInfo, "/oakd/rgb/preview/camera_info", lambda m: stamped("camera_info", m), qos_profile_sensor_data),
            self.node.create_subscription(Odometry, "/odom", lambda m: stamped("odom", m, (m.twist.twist.linear.x, m.twist.twist.angular.z)), qos_profile_sensor_data),
            self.node.create_subscription(HazardDetectionVector, "/hazard_detection", lambda m: self.sensors.update("hazards", time.monotonic(), value=[v.type for v in m.detections]), qos_profile_sensor_data),
            self.node.create_subscription(DockStatus, "/dock_status", lambda m: self.sensors.update("dock", time.monotonic(), value=m.is_docked), qos_profile_sensor_data),
            self.node.create_subscription(String, prefix + "/status", self.on_status, status_qos),
        ]

    def on_status(self, msg):
        try:
            parsed = json.loads(msg.data)
            if isinstance(parsed, dict):
                self.status = normalize_r32(parsed) if self.runtime == "upstream-r32" else parsed
                self.status_at = time.monotonic()
        except ValueError:
            pass

    def spin(self, timeout=0.05):
        self.rclpy.spin_once(self.node, timeout_sec=timeout)

    def safety_reason(self):
        return self.sensors.reason(time.monotonic(), self.node.get_clock().now().nanoseconds / 1e9)

    def conflicts(self):
        return sorted(name for name, ns in self.node.get_node_names_and_namespaces()
                      if ns == "/" and name in CONFLICTS)

    def zero(self):
        if self.zero_pub is None:
            self.zero_pub = self.node.create_publisher(self.TwistStamped, "/cmd_vel", 10)
        msg = self.TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        self.zero_pub.publish(msg)

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def clock_check(server):
    def query(*args):
        return subprocess.run(["timedatectl", *args], capture_output=True, text=True,
                              check=True, timeout=3).stdout.strip()
    if query("show", "--property=NTPSynchronized", "--value") != "yes":
        raise RuntimeError("VM clock is not NTP synchronized; start refused")
    actual = query("show-timesync", "--property=ServerAddress", "--value")
    if actual != server:
        raise RuntimeError("VM NTP server is " + actual + "; expected robot " + server)


def upstream_config(workspace):
    return workspace / "install/osp_project/share/osp_project/resources/mission_r32_hardware.yaml"


def runtime_check(args):
    from ament_index_python.packages import get_package_prefix
    native = getattr(args, "runtime", "flow") == "upstream-r32"
    package = "osp_project" if native else "osp_mission"
    expected = args.workspace / "install" / package
    if Path(get_package_prefix(package)).resolve() != expected.resolve():
        raise RuntimeError("Wrong " + package + " overlay; source the prepared workspace")
    files = [upstream_config(args.workspace)] if native else [
        args.repo / "robot_profiles/wired_flow_v3/nav2_hardware_profile.yaml",
        args.repo / "policy_releases/2026-11-13_face_intent_v2/policy.json"]
    for path in [args.workspace / "install/setup.bash", *files]:
        if not path.is_file():
            raise FileNotFoundError("Prepare this file before start: " + str(path))
    profile = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", "")
    if not profile or not Path(profile).is_file():
        raise RuntimeError("Wired Fast DDS profile missing")
    if os.environ.get("RMW_IMPLEMENTATION") != "rmw_fastrtps_cpp":
        raise RuntimeError("Expected wired rmw_fastrtps_cpp configuration")


def launch_command(args, output):
    if getattr(args, "runtime", "flow") == "upstream-r32":
        return ["ros2", "launch", "osp_project", "mission.launch.py",
                "config:=" + str(upstream_config(args.workspace)), "output_directory:=" + str(output / "upstream"),
                "use_sim_time:=false", "bringup:=true", "rviz:=false"]
    return ["ros2", "launch", "osp_mission", "robot.launch.py", "use_sim_time:=false",
            "rviz:=false", "planner:=edge", "planner_start_ignore_radius_m:=0.30",
            "navigation_flow_enabled:=true", "allow_mid_route_wall_inspection:=false",
            "undock:=false", "duration:=" + str(args.duration),
            "nav2_params:=" + str(args.repo / "robot_profiles/wired_flow_v3/nav2_hardware_profile.yaml"),
            "policy_file:=" + str(args.repo / "policy_releases/2026-11-13_face_intent_v2/policy.json"),
            "output_dir:=" + str(output)]


def identity(pid):
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return None if fields[0] == "Z" else fields[19]
    except (OSError, IndexError):
        return None


def owned_alive(state, prefix):
    boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    pid, expected = state.get(prefix + "_pid"), state.get(prefix + "_start")
    return bool(state.get("boot_id") == boot and pid and expected is not None and identity(pid) == expected)


def group_active(pgid):
    """Count live children too, even if the launch group leader already exited."""
    for path in Path("/proc").glob("[0-9]*/stat"):
        try:
            fields = path.read_text().rsplit(")", 1)[1].split()
            if fields[0] != "Z" and fields[2] == str(pgid):
                return True
        except (OSError, IndexError):
            continue
    return False


def tracked_group_active(state):
    pid = state.get("launch_pid")
    boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    if not pid or state.get("boot_id") != boot or state.get("launch_start") is None:
        return False
    current = identity(pid)
    if current is not None and current != state["launch_start"]:
        return False  # PID reused by an unrelated process.
    return group_active(pid)


def save(path, value):
    temp = path.with_suffix(".tmp")
    temp.write_text(json.dumps(value, indent=2) + "\n")
    temp.replace(path)


def event(output, started, name, **fields):
    row = dict(event=name, elapsed_s=round(time.monotonic() - started, 3),
               wall_epoch_s=time.time(), **fields)
    with (output / "startup.jsonl").open("a") as stream:
        stream.write(json.dumps(row) + "\n")
    print(json.dumps(row), flush=True)


def preflight(io, flag, timeout):
    deadline = time.monotonic() + timeout
    reason = "no_sensor_messages"
    while time.monotonic() < deadline:
        if flag.stopped:
            raise InterruptedError("operator_stop")
        io.spin()
        reason = io.safety_reason()
        if reason.startswith(("robot_docked", "active_hazard")):
            raise RuntimeError(reason)
        if not reason:
            conflicts = io.conflicts()
            if conflicts:
                raise RuntimeError("Existing navigation nodes; stop that session first: " + ",".join(conflicts))
            return
    raise TimeoutError("Preflight: " + reason)


def repair_camera(io, flag, report):
    # Only before launching our stack, with measured stationary wheels.
    if flag.stopped or io.conflicts() or not io.sensors.stationary(time.monotonic()):
        raise RuntimeError("Camera repair requires stopped robot and no active navigation")
    report("camera_reinitialization", reason="stale_capture_timestamp")
    for service in ("/oakd/stop_camera", "/oakd/start_camera"):
        client = io.node.create_client(io.Trigger, service)
        future = None
        try:
            deadline = time.monotonic() + 12.
            while not client.service_is_ready():
                if flag.stopped or time.monotonic() > deadline:
                    raise RuntimeError("Camera service unavailable or operator stop")
                io.spin()
            future = client.call_async(io.Trigger.Request())
            while not future.done():
                if flag.stopped or time.monotonic() > deadline:
                    raise RuntimeError("Camera repair interrupted or timed out")
                io.spin()
            if not future.result().success:
                raise RuntimeError("Camera repair rejected: " + service)
        finally:
            if future is not None and not future.done():
                future.cancel()
                client.remove_pending_request(future)
            io.node.destroy_client(client)


def wait_for_start(io, flag, process, timeout, report):
    gate = StartGate()
    deadline = time.monotonic() + timeout
    previous_reason = None
    try:
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("ROS launch exited before mission start")
            io.spin()
            reason = io.safety_reason()
            state = io.status.get("state", "") if time.monotonic() - io.status_at < 2 else ""
            if getattr(io, "runtime", "flow") == "upstream-r32" and state == "idle":
                reason = reason or io.status.get("readiness_reason")
            detail = reason or io.status.get("readiness_reason", "waiting_for_mission_service")
            if detail != previous_reason:
                report("waiting", reason=detail)
                previous_reason = detail
            if gate.step(time.monotonic(), flag.stopped, reason, state,
                         io.start_client, io.Trigger.Request):
                report("mission_started")
                return
        raise TimeoutError("Mission readiness timed out: " + str(previous_reason))
    finally:
        gate.cancel(io.start_client)


def stop_motion(io, seconds=3.0):
    deadline = time.monotonic() + seconds
    pending = {}
    while time.monotonic() < deadline:
        io.zero()
        clients = {**io.cancel_clients, "mission_stop": io.stop_client}
        for name, client in clients.items():
            if name not in pending and client.service_is_ready():
                request = io.Trigger.Request() if name == "mission_stop" else io.CancelGoal.Request()
                pending[name] = client.call_async(request)
        io.spin()
    return io.sensors.stationary(time.monotonic())


def end_launch(process):
    for sig, seconds in ((signal.SIGINT, 8), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
        if process.poll() is not None and not group_active(process.pid):
            return
        deadline = time.monotonic() + seconds
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            process.poll()
            return
        try:
            process.wait(timeout=seconds)
        except subprocess.TimeoutExpired:
            continue
        while group_active(process.pid) and time.monotonic() < deadline:
            time.sleep(0.05)
        if not group_active(process.pid):
            return
    raise RuntimeError("Launch process group did not stop")


def start(args, io, flag, state_path):
    started = time.monotonic()
    output = Path(os.environ.get("OSP_RUN_DIR", str(args.workspace / "session_logs" / str(time.time_ns())))) / "artifacts"
    output.mkdir(parents=True, exist_ok=True)
    report = lambda name, **fields: event(output, started, name, **fields)
    process = None
    state = dict(supervisor_pid=os.getpid(), supervisor_start=identity(os.getpid()),
                 boot_id=Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
                 phase="preflight", workspace=str(args.workspace), runtime=getattr(args, "runtime", "flow"),
                 output=str(output), domain=args.domain, requested_wall_epoch_s=time.time())
    save(state_path, state)
    try:
        report("entry")
        runtime_check(args)
        clock_check(args.ntp_server)
        try:
            preflight(io, flag, min(args.timeout, 8.0))
        except TimeoutError as exc:
            if "camera" not in str(exc):
                raise
            repair_camera(io, flag, report)
            preflight(io, flag, min(args.timeout, 8.0))
        report("preflight_passed")
        if flag.stopped:
            raise InterruptedError("operator_stop")
        process = subprocess.Popen(launch_command(args, output), start_new_session=True)
        state.update(phase="starting", launch_pid=process.pid, launch_start=identity(process.pid))
        save(state_path, state)
        report("launch_started")
        wait_for_start(io, flag, process, args.timeout, report)
        state["phase"] = "running"
        save(state_path, state)
        first_motion = first_nav = first_forward = False
        running_since = time.monotonic()
        bump_guard = BumpRecoveryGuard()
        while not flag.stopped:
            io.spin()
            if process.poll() is not None:
                raise RuntimeError("ROS launch exited during mission")
            hazards = io.sensors.samples.get("hazards")
            if hazards:
                before = bump_guard.total
                reason = bump_guard.check(time.monotonic(), hazards[2])
                if reason:
                    report("safety_stop", reason=reason, hazard_types=hazards[2])
                    raise RuntimeError(reason)
                if bump_guard.total != before:
                    report("bumper_recovery", contacts=bump_guard.total,
                           action="mission_cancel_safe_backup_replan")
            if not first_motion and io.sensors.odom and any(abs(v) > 0.01 for v in io.sensors.odom[-1][1:]):
                report("first_motion")
                first_motion = True
            if not first_forward and io.sensors.odom:
                received, linear, _ = io.sensors.odom[-1]
                if received >= running_since and linear > .02:
                    report("first_forward_odometry", linear_m_s=round(linear, 3))
                    first_forward = True
            if not first_nav and io.status.get("active_action") == "nav":
                report("first_navigation")
                first_nav = True
            if io.status.get("state") == "done":
                report("mission_finished", termination_reason=io.status.get("termination_reason"))
                break
        return 0
    finally:
        state["phase"] = "stopping" if process else "preflight_ended"
        save(state_path, state)
        if process is not None:
            # Give the native recorder a stationary final status while its nodes are alive.
            if state["runtime"] == "upstream-r32" and process.poll() is None:
                stop_motion(io, 3.0)
            # Signal launch immediately; keep zero/cancel coverage during teardown.
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
            try:
                stop_motion(io, 3.0)
            finally:
                end_launch(process)
            stopped = stop_motion(io, 1.2)
            report("stopped", stationary_verified=stopped)
            state["stationary_verified"] = stopped
        state["phase"] = ("stopped" if stopped else "stop_unverified") if process else "not_started"
        save(state_path, state)
        if process is not None and not stopped:
            raise RuntimeError("Launch ended but stationary odometry could not be verified")


def read_state(path):
    try:
        return json.loads(path.read_text())
    except FileNotFoundError:
        return {}


def stop(args, io, state_path):
    state = read_state(state_path)
    if owned_alive(state, "supervisor"):
        try:
            os.kill(state["supervisor_pid"], signal.SIGTERM)
        except ProcessLookupError:
            pass
    if state.get("runtime") == "upstream-r32" and tracked_group_active(state):
        stop_motion(io, 3.0)
    if tracked_group_active(state):
        try:
            os.killpg(state["launch_pid"], signal.SIGINT)
        except ProcessLookupError:
            pass
    stationary = stop_motion(io, 6.0)
    for sig in (signal.SIGTERM, signal.SIGKILL):
        if not tracked_group_active(state):
            break
        try:
            os.killpg(state["launch_pid"], sig)
        except ProcessLookupError:
            pass
        stationary = stop_motion(io, 1.2)
    launch_alive = tracked_group_active(state)
    print(json.dumps(dict(stationary_verified=stationary, tracked_launch_alive=launch_alive,
                          odometry=list(io.sensors.odom)[-5:])), flush=True)
    return 0 if stationary and not launch_alive else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("start", "stop", "check", "status", "dry-run"))
    parser.add_argument("--runtime", choices=("flow", "upstream-r32"), default="flow")
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--workspace", type=Path, default=Path.home() / "face_intent_flow_v4r2_ws")
    parser.add_argument("--domain", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "4")))
    parser.add_argument("--ntp-server", default="192.168.185.3")
    parser.add_argument("--duration", type=float, default=600.0)
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232 or not all(math.isfinite(v) and v > 0 for v in (args.duration, args.timeout)):
        parser.error("Valid ROS domain and positive finite duration/timeout required")
    if args.action == "dry-run":
        print(json.dumps(launch_command(args, Path("RUN_DIR/artifacts")), indent=2))
        return 0
    state_dir = Path.home() / ".local/state/osp-robot" / ("domain-" + str(args.domain))
    state_dir.mkdir(parents=True, exist_ok=True)
    state_path = state_dir / "session.json"
    flag = StopFlag()
    lock = None
    if args.action == "start":
        lock = (state_dir / "session.lock").open("w")
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            lock.close()
            print("Another robot session owns this domain; start refused", flush=True)
            return 1
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: setattr(flag, "stopped", True))
    io = None
    try:
        if args.action == "stop":
            args.runtime = read_state(state_path).get("runtime", "flow")
        io = RobotIO(args.domain, args.runtime)
        if args.action == "start":
            return start(args, io, flag, state_path)
        if args.action == "stop":
            return stop(args, io, state_path)
        if args.action == "check":
            runtime_check(args)
            clock_check(args.ntp_server)
            began = time.monotonic()
            preflight(io, flag, min(args.timeout, 8.0))
            print(json.dumps(dict(check="passed", elapsed_s=round(time.monotonic() - began, 3),
                                  motion_commands_sent=False, runtime_built=False)), flush=True)
            return 0
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            io.spin()
        state = read_state(state_path)
        print(json.dumps(dict(session=state, supervisor_alive=owned_alive(state, "supervisor"),
                              tracked_launch_alive=tracked_group_active(state), mission=io.status,
                              sensor_reason=io.safety_reason(), stationary=io.sensors.stationary(time.monotonic()),
                              navigation_nodes=io.conflicts())), flush=True)
        return 0
    except (RuntimeError, OSError, TimeoutError, InterruptedError, subprocess.SubprocessError) as exc:
        print(type(exc).__name__ + ": " + str(exc), flush=True)
        return 1
    finally:
        if io is not None:
            io.close()
        if lock is not None:
            lock.close()


if __name__ == "__main__":
    raise SystemExit(main())
