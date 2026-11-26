#!/usr/bin/env bash
# A replay never writes to the baseline run. The enclosing campaign owns the native benchmark lock.
set -eo pipefail
: "${1:?baseline recording}" "${2:?ROS simulation domain}" "${3:?new output directory}" "${4:?candidate overlay}"
[[ "$2" == 173 || "$2" == 174 || "$2" == 175 ]]
base=$(realpath "$1")
mkdir "$3"
export BENCH_RUN=$(realpath "$3")
export BENCH_ROOT="$BENCH_RUN"
for f in scene.yaml scene.json world run; do test -e "$base/$f"; ln -s "$base/$f" "$BENCH_RUN/$f"; done
tools_dir="$HOME/track-resume-5e95333"
source /opt/ros/jazzy/setup.bash
source "$HOME/flow_v3_r32_adaptive_ws/install/setup.bash"
export PYTHONPATH="$(realpath "$4"):${PYTHONPATH:-}"
BENCH_POLICY=$(python3 - "$(realpath "$4")" <<'PYTHON'
from pathlib import Path
import sys
from osp_mission import mission, detector, marker_nbv
root=Path(sys.argv[1])
if not all(Path(mod.__file__).resolve().is_relative_to(root) for mod in (mission,detector,marker_nbv)):
    raise SystemExit('Simulation resolved a module outside the selected overlay')
source=Path(mission.__file__).read_text()
imports={'from osp_mission.guarded_nbv import GuardedNBV as Hybrid':'marker_nbv_local_v1_1',
         'from osp_mission.marker_nbv import MarkerNBV as Hybrid':'marker_nbv_v1'}
matches=[version for statement,version in imports.items() if statement in source]
if len(matches)!=1:
    raise SystemExit('Cannot identify the staged policy')
print(matches[0])
PYTHON
)
export BENCH_POLICY
[[ -z "${NBV_VERSION:-}" || "$BENCH_POLICY" == "$NBV_VERSION" ]] || { echo 'Version and overlay differ' >&2; exit 2; }
export ROS_DOMAIN_ID="$2" ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="file://$tools_dir/2026-11-19_pair_7_103/cyclone.xml"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ROS_HOME="$BENCH_RUN/ros_home" ROS_LOG_DIR="$BENCH_RUN/ros_logs"
unset ROS_STATIC_PEERS ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
echo "$BENCH_POLICY" > "$BENCH_RUN/policy_revision.txt"
python3 -u "$tools_dir/2026-11-19_corridor_box/measure_video.py" > "$BENCH_RUN/observer.log" 2>&1
python3 "$tools_dir/2026-11-19_corridor_box/finish_trial.py" "$BENCH_RUN" "$BENCH_RUN" >> "$BENCH_RUN/observer.log" 2>&1
