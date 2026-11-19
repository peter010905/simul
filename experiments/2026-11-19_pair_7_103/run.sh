#!/usr/bin/env bash
set -eo pipefail
export BENCH_ROOT="$(cd "$(dirname "$0")" && pwd)"
export BENCH_POLICY="${1:?flow, mixed or r32}" BENCH_RUN="${2:?fresh output directory}"
case "$BENCH_POLICY" in
  flow|r32) ws="$HOME/face_intent_v3_sweep_ws" ;;
  mixed) ws="$HOME/flow_v3_r32_print_ws" ;;
  *) exit 2 ;;
esac
source /opt/ros/jazzy/setup.bash
source "$ws/install/setup.bash"
source "$HOME/r32_upstream_b1ed300_ws/install/setup.bash"
export ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="file://$BENCH_ROOT/cyclone.xml"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
unset ROS_STATIC_PEERS ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
mkdir "$BENCH_RUN"
export ROS_HOME="$BENCH_RUN/ros_home" ROS_LOG_DIR="$BENCH_RUN/ros_logs"
python3 - "$BENCH_ROOT" <<'PY'
from pathlib import Path
import sys,yaml
from osp_mission.world import generate
from ament_index_python.packages import get_package_share_directory as share
root=Path(sys.argv[1])
if not (root/'world/osp_studio.sdf').exists():
    generate(root/'scene.yaml',root/'world')
cfg=yaml.safe_load((Path(share('osp_project'))/'resources/mission_r32_hardware.yaml').read_text())
cfg['mission']['ros__parameters'].update(mission_time_source='ros',duration=120.0)
(root/'r32-sim.yaml').write_text(yaml.safe_dump(cfg))
PY
exec python3 -u "$BENCH_ROOT/measure.py"
