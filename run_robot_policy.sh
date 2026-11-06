#!/usr/bin/env bash
# TurtleBot4 + ROS 2 Jazzy. Run from the robot's onboard or networked computer.
set -eo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
runtime_dir="$repo_dir/deployment/robot_v13"
domain="${ROS_DOMAIN_ID:-}"
namespace=""
rgb_topic="oakd/rgb/preview/image_raw"
camera_info_topic="oakd/rgb/preview/camera_info"
scan_topic="scan"
duration="600"
build=false
build_only=false
check=false
dry_run=false
start=false
rviz=false
undock=false

usage() {
    cat <<'HELP'
Usage: bash run_robot_policy.sh --domain ROBOT_DOMAIN [options]

  --build              Copy v13 and build an isolated hardware runtime
  --build-only         Build, then exit without connecting to the robot
  --check              Check robot discovery and topic types, then exit
  --dry-run            Print the launch command without connecting or moving
  --start              Start the mission automatically after readiness checks
  --namespace NAME     Robot ROS namespace (default empty)
  --rgb TOPIC          RGB Image (default oakd/rgb/preview/image_raw)
  --camera-info TOPIC   Matching CameraInfo
  --scan TOPIC          LaserScan (default scan)
  --duration SECONDS    Mission duration (default 600)
  --rviz               Open RViz
  --undock             Undock at mission start (default false)
  -h, --help           Show this help

Requires the repository, ROS 2 Jazzy, colcon, and package dependencies.
Start TurtleBot4 base/sensors bringup first. This script starts SLAM and Nav2.
Use the robot's ROS_DOMAIN_ID, RMW and discovery settings on both computers.
Without --start the mission stays idle. Start/stop services are printed below.
The v13 candidate includes known observation/identity limitations and has not
been validated on physical hardware. This launcher uses TwistStamped.
HELP
}

fail() { printf '%s\n' "$*" >&2; exit 1; }
while (($#)); do
    case "$1" in
        --domain|--namespace|--rgb|--camera-info|--scan|--duration)
            (($# >= 2)) || fail "Missing value for $1"
            case "$1" in
                --domain) domain="$2" ;;
                --namespace) namespace="$2" ;;
                --rgb) rgb_topic="$2" ;;
                --camera-info) camera_info_topic="$2" ;;
                --scan) scan_topic="$2" ;;
                --duration) duration="$2" ;;
            esac
            shift 2 ;;
        --build) build=true; shift ;;
        --build-only) build=true; build_only=true; shift ;;
        --check) check=true; shift ;;
        --dry-run) dry_run=true; shift ;;
        --start) start=true; shift ;;
        --rviz) rviz=true; shift ;;
        --undock) undock=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) fail "Unknown argument $1" ;;
    esac
done
[[ "$domain" =~ ^[0-9]{1,3}$ ]] && ((10#$domain <= 232)) || fail 'Specify --domain with the robot ROS domain (0..232).'
[[ "$duration" =~ ^[0-9]+([.][0-9]+)?$ ]] && [[ "$duration" =~ [1-9] ]] || fail 'Duration must be positive.'
namespace="${namespace#/}"
namespace="${namespace%/}"
export ROS_DOMAIN_ID="$domain"
export OPENBLAS_NUM_THREADS=1

source_dir="$repo_dir/training_robust/native90/runtime_fix_v13_observation_staging/src/osp_mission"
[[ -f "$source_dir/osp_mission/mission.py" ]] || fail "Missing v13 source at $source_dir"
command=(ros2 launch osp_mission robot.launch.py
    use_sim_time:=false require_drive_controller:=false planner:=edge
    "namespace:=$namespace" "rgb_topic:=$rgb_topic" "camera_info_topic:=$camera_info_topic"
    depth_topic:=unused_depth "scan_topic:=$scan_topic" "duration:=$duration"
    "undock:=$undock" "rviz:=$rviz"
    "policy_file:=$runtime_dir/install/osp_mission/share/osp_mission/config/edge_policy.json"
    "nav2_params:=$runtime_dir/nav2_robot_active.yaml"
    "output_dir:=$repo_dir/results/robot_$(date +%Y%m%dT%H%M%S)_$$")
service_prefix="/${namespace:+$namespace/}mission"
printf 'Candidate v13 hardware adapter, ROS_DOMAIN_ID=%s\n' "$domain"
printf 'Launch command '; printf '%q ' "${command[@]}"; printf '\n'
printf 'Start: ROS_DOMAIN_ID=%s ros2 service call %s/start std_srvs/srv/Trigger "{}"\n' "$domain" "$service_prefix"
printf 'Stop:  ROS_DOMAIN_ID=%s ros2 service call %s/stop std_srvs/srv/Trigger "{}"\n' "$domain" "$service_prefix"
if $dry_run; then exit 0; fi

[[ -f /opt/ros/jazzy/setup.bash ]] || fail 'ROS 2 Jazzy is required at /opt/ros/jazzy.'
source /opt/ros/jazzy/setup.bash
if $build; then
    mkdir -p "$runtime_dir"
    exec 9>"$runtime_dir/build.lock"
    flock -n 9 || fail 'This runtime is already being built or used.'
    /usr/bin/python3 "$repo_dir/scripts/prepare_robot_runtime.py" \
        --source "$source_dir" --destination "$runtime_dir"
    colcon --log-base "$runtime_dir/log" build \
        --base-paths "$runtime_dir/src/osp_mission" --packages-select osp_mission \
        --build-base "$runtime_dir/build" --install-base "$runtime_dir/install"
fi
if $build_only; then exit 0; fi
[[ -f "$runtime_dir/install/local_setup.bash" ]] || fail 'Build first with --build or --build-only.'
if ! $build; then
    exec 9>"$runtime_dir/build.lock"
    flock -n 9 || fail 'This runtime is already being built or used.'
fi
source "$runtime_dir/install/local_setup.bash"
[[ "$(ros2 pkg prefix osp_mission)" == "$runtime_dir/install/osp_mission" ]] || fail 'A different osp_mission overlay is active.'
/usr/bin/python3 - "$runtime_dir" "$scan_topic" <<'PY'
from pathlib import Path
import sys
import yaml

runtime = Path(sys.argv[1])
config = yaml.safe_load((runtime / 'install/osp_mission/share/osp_mission/config/nav2_robot_policy.yaml').read_text())
for key in ('local_costmap', 'global_costmap'):
    parameters = config[key][key]['ros__parameters']
    for layer in ('voxel_layer', 'obstacle_layer'):
        if layer in parameters:
            parameters[layer]['scan']['topic'] = sys.argv[2]
config['collision_monitor']['ros__parameters']['scan']['topic'] = sys.argv[2]
(runtime / 'nav2_robot_active.yaml').write_text(yaml.safe_dump(config))
PY

# Read-only graph checks. The mission itself additionally checks fresh messages,
# camera calibration, TF, the map and active Nav2 lifecycle states before start.
/usr/bin/python3 - "$namespace" "$rgb_topic" "$camera_info_topic" "$scan_topic" <<'PY'
import sys
import time
import rclpy

namespace, rgb, info, scan = sys.argv[1:]
rclpy.init()
node = rclpy.create_node('robot_policy_preflight', namespace='/' + namespace)
try:
    deadline = time.monotonic() + 8
    required = {node.resolve_topic_name(topic): kind for topic, kind in (
        (rgb, 'sensor_msgs/msg/Image'), (info, 'sensor_msgs/msg/CameraInfo'),
        (scan, 'sensor_msgs/msg/LaserScan'), ('odom', 'nav_msgs/msg/Odometry'))}
    while True:
        rclpy.spin_once(node, timeout_sec=0.2)
        topics = dict(node.get_topic_names_and_types())
        missing = [f'{topic} ({kind})' for topic, kind in required.items()
                   if kind not in topics.get(topic, [])]
        if not missing or time.monotonic() >= deadline:
            break
    if missing:
        raise SystemExit('Robot topics unavailable. Check bringup/domain/RMW/discovery: ' + ', '.join(missing))
    cmd = node.resolve_topic_name('cmd_vel')
    types = topics.get(cmd, [])
    if types and types != ['geometry_msgs/msg/TwistStamped']:
        raise SystemExit(f'{cmd} has {types}. This runtime requires TwistStamped.')
    names = {('/' + ns.strip('/') + '/' + name).replace('//', '/')
             for name, ns in node.get_node_names_and_namespaces()}
    conflicts = names.intersection(node.resolve_topic_name(name) for name in (
        'slam_toolbox', 'controller_server', 'planner_server', 'bt_navigator',
        'mission', 'aruco_detector', 'collision_monitor', 'velocity_smoother'))
    if conflicts:
        raise SystemExit('SLAM/Nav2/mission already running in this namespace: ' + ', '.join(sorted(conflicts)))
    print('Robot topic types and duplicate-node checks passed.')
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
if $check; then exit 0; fi

launch_pid=""
cleanup() {
    trap - EXIT INT TERM
    if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
        kill -INT "$launch_pid" 2>/dev/null || true
        wait "$launch_pid" || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
"${command[@]}" &
launch_pid=$!
if $start; then
    /usr/bin/python3 - "$service_prefix/start" "$launch_pid" <<'PY'
import os
import sys
import time
import rclpy
from std_srvs.srv import Trigger

rclpy.init()
node = rclpy.create_node('robot_policy_starter')
try:
    client = node.create_client(Trigger, sys.argv[1])
    deadline = time.monotonic() + 120
    while time.monotonic() < deadline:
        os.kill(int(sys.argv[2]), 0)
        if not client.wait_for_service(timeout_sec=1):
            continue
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done():
            # Avoid an outstanding start request being delivered after a retry.
            raise SystemExit('Mission start RPC timed out. Restart and check the runtime.')
        reply = future.result()
        if reply.success:
            print(reply.message)
            break
        print(reply.message, flush=True)
        time.sleep(1)
    else:
        raise SystemExit('Mission readiness timed out after 120 seconds.')
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
fi
wait "$launch_pid"
