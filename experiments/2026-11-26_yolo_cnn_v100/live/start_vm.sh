#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
fixture="${1:-$HOME/classroom-twin/scene16-parity}"
workspace="${2:-$HOME/twin43_hardware_ws}"

exec 9>"${XDG_RUNTIME_DIR:-/tmp}/osp-marker-board-domain-86.lock"
flock -n 9
test -s "$fixture/world/osp_studio.sdf"
test -s bridge-token
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_DISCOVERY_SERVER FASTDDS_DEFAULT_PROFILES_FILE FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=86 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp GZ_PARTITION="osp_marker_board_$$"
export CYCLONEDDS_URI="file://$PWD/cyclone.xml"
export LIBGL_ALWAYS_SOFTWARE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 LP_NUM_THREADS=2
export ROS_HOME="$PWD/ros_home" ROS_LOG_DIR="$PWD/ros_logs"
export GZ_SIM_RESOURCE_PATH="$(dirname "$(ros2 pkg prefix --share turtlebot4_description)"):$(dirname "$(ros2 pkg prefix --share irobot_create_description)")"
export CAMERA_BENCH_WIDTH=250 CAMERA_BENCH_HEIGHT=250 CAMERA_BENCH_FPS=5
read -r x y yaw < <(python3 -c 'import sys,yaml; print(*yaml.safe_load(open(sys.argv[1]))["start"])' "$fixture/scene.yaml")

children=()
start() {
    setsid "$@" </dev/null >>"$run_log" 2>&1 &
    children+=("$!")
}
cleanup() {
    for child in "${children[@]}"; do kill -TERM -- "-$child" 2>/dev/null || true; done
    wait || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
run_log=gazebo.log
start gz sim "$fixture/world/osp_studio.sdf" -s -r -v 2 --headless-rendering
sleep 3
run_log=clock.log
start ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
run_log=spawn.log
start ros2 launch osp_mission spawn.launch.py model:=standard world:=osp_studio camera_width:=250 start_docked:=false \
    x:="$x" y:="$y" yaw:="$yaw"
run_log=bridge.log
start python3 -u bridge.py
wait -n "${children[@]}"
