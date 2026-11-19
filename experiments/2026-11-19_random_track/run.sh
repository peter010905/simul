#!/usr/bin/env bash
set -eo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
reference="${TRACK_REFERENCE:-$HOME/track-reference-73bbb05}"
runs="${TRACK_RUNS:-$HOME/track-recordings}"
export BENCH_POLICY="${1:-mixed}"
case "$BENCH_POLICY" in
  flow) ws="$HOME/face_intent_v3_sweep_ws" ;;
  mixed) ws="$HOME/flow_v3_r32_print_ws" ;;
  *) echo 'Choose mixed or flow' >&2; exit 2 ;;
esac
mkdir -p "$runs"
# A shared simulation domain has exactly one control client at a time.
exec 9>"$runs/.domain.lock"
flock -n 9 || { echo 'A track trial is already running' >&2; exit 75; }
export BENCH_RUN="$(mktemp -d "$runs/$BENCH_POLICY-track-XXXXXXXX")"
export BENCH_ROOT="$BENCH_RUN"
source /opt/ros/jazzy/setup.bash
source "$ws/install/setup.bash"
export PYTHONPATH="$reference:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="file://$here/../2026-11-19_pair_7_103/cyclone.xml"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
unset ROS_STATIC_PEERS ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_HOME="$BENCH_RUN/ros_home" ROS_LOG_DIR="$BENCH_RUN/ros_logs"
echo "$BENCH_RUN"
python3 "$here/prepare.py" "$reference" "$BENCH_RUN"
python3 -u "$here/../2026-11-19_corridor_box/measure_video.py"
python3 "$here/../2026-11-19_corridor_box/finish_trial.py" "$BENCH_RUN" "$BENCH_RUN"
