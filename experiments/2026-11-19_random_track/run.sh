#!/usr/bin/env bash
set -eo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
reference="${TRACK_REFERENCE:-$HOME/track-reference-73bbb05}"
runs="${TRACK_RUNS:-$HOME/track-recordings}"
mode="${1:-pair}"
case "$mode" in flow|mixed|pair) ;; *) echo 'Choose pair, mixed or flow' >&2; exit 2 ;; esac
mkdir -p "$runs/.layouts"
exec 9>"$runs/.domain.lock"
flock -n 9 || { echo 'A track trial is already running' >&2; exit 75; }
source /opt/ros/jazzy/setup.bash
layout="$(mktemp -d "$runs/.layouts/XXXXXXXX")"
PYTHONPATH="$reference:${PYTHONPATH:-}" python3 "$here/prepare.py" "$reference" "$layout"

drive() (
    export BENCH_POLICY="$1" ROS_DOMAIN_ID="$2"
    export BENCH_RUN="$runs/$BENCH_POLICY-track-${layout##*/}"
    export BENCH_ROOT="$BENCH_RUN"
    mkdir "$BENCH_RUN"
    ln -s "$layout"/{scene.json,scene.yaml,run,world} "$BENCH_RUN/"
    if [[ "$BENCH_POLICY" == mixed ]]; then
        source "$HOME/flow_v3_r32_print_ws/install/setup.bash"
        if [[ -n "${TRACK_MIXED_OVERLAY:-}" ]]; then
            test -f "$TRACK_MIXED_OVERLAY/osp_mission/box_views.py"
            export PYTHONPATH="$TRACK_MIXED_OVERLAY:${PYTHONPATH:-}"
            basename "$TRACK_MIXED_OVERLAY" > "$BENCH_RUN/policy_revision.txt"
        fi
    else
        source "$HOME/face_intent_v3_sweep_ws/install/setup.bash"
    fi
    if [[ "$BENCH_POLICY" == r32 ]]; then
        source "$HOME/r32_upstream_73bbb05_ws/install/local_setup.bash"
        if [[ -n "${TRACK_RECORDER_OVERLAY:-}" ]]; then
            export PYTHONPATH="$TRACK_RECORDER_OVERLAY:${PYTHONPATH:-}"
        fi
        python3 - "$BENCH_ROOT/r32-sim.yaml" <<'PY'
from pathlib import Path
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
config = Path(get_package_share_directory('osp_project'))/'resources/mission_r32_hardware.yaml'
settings = yaml.safe_load(config.read_text())
settings['mission']['ros__parameters'].update(mission_time_source='ros', duration=600.)
Path(sys.argv[1]).write_text(yaml.safe_dump(settings))
PY
    fi
    export ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="file://$here/../2026-11-19_pair_7_103/cyclone.xml"
    export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ROS_HOME="$BENCH_RUN/ros_home" ROS_LOG_DIR="$BENCH_RUN/ros_logs"
    unset ROS_STATIC_PEERS ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
    python3 -u "$here/../2026-11-19_corridor_box/measure_video.py" >"$BENCH_RUN/observer.log" 2>&1
    python3 "$here/../2026-11-19_corridor_box/finish_trial.py" "$BENCH_RUN" "$BENCH_RUN" >>"$BENCH_RUN/observer.log" 2>&1
)

if [[ "$mode" == pair ]]; then
    drive mixed 174 & mixed_pid=$!
    drive r32 175 & r32_pid=$!
    result=0
    wait "$mixed_pid" || result=$?
    wait "$r32_pid" || result=$?
    exit "$result"
fi
drive "$mode" 174
