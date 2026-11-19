#!/usr/bin/env bash
set -eo pipefail
export BENCH_ROOT="$(cd "$(dirname "$0")" && pwd)"
export BENCH_POLICY="${1:?flow or mixed}" BENCH_RUN="${2:?new absolute output directory}"
case "$BENCH_POLICY" in
 flow) ws="$HOME/face_intent_v3_sweep_ws" ;;
 mixed) ws="$HOME/flow_v3_r32_print_ws" ;;
 *) exit 2 ;;
esac
source /opt/ros/jazzy/setup.bash
source "$ws/install/setup.bash"
export ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="file://$BENCH_ROOT/../2026-11-19_pair_7_103/cyclone.xml"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
unset ROS_STATIC_PEERS ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
mkdir "$BENCH_RUN"
export ROS_HOME="$BENCH_RUN/ros_home" ROS_LOG_DIR="$BENCH_RUN/ros_logs"
python3 - "$BENCH_ROOT" <<'PY'
from pathlib import Path
import sys
import yaml
from osp_mission.world import generate
root=Path(sys.argv[1])
assert yaml.safe_load((root/'scene.yaml').read_text())['start'] == [3.24,2.40,0.]
if not (root/'world/osp_studio.sdf').exists():
    generate(root/'scene.yaml',root/'world')
PY
exec python3 -u "$BENCH_ROOT/measure_video.py"
