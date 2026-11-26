#!/usr/bin/env bash
# Replay exactly one recorded scene with the isolated simulation policy.
set -eo pipefail
[[ "$1" =~ ^shared-track-[a-zA-Z0-9]+$ ]] || exit 2
runs="$HOME/track-recordings"
base="$runs/$1"
case "${2:-batch}" in
    batch) mode=batch; revision=flow_v3_r32_box_batch_ws ;;
    wall-cues) mode=wall-cues; revision=flow_v3_r32_wall_cues_final_ws ;;
    *) exit 2 ;;
esac
trial="$runs/$mode-${1#shared-}"
suite="$HOME/track-resume-5e95333"
exec 9>"$runs/.domain.lock"
flock 9
source /opt/ros/jazzy/setup.bash
source "$HOME/$revision/install/setup.bash"
python3 -c 'import sys; from pathlib import Path; from ament_index_python.packages import get_package_share_directory as share; from osp_mission.box_batch import BoxBatch; assert sys.argv[1] in Path(share("osp_mission")).parts' "$revision"
if [[ "$mode" == wall-cues ]]; then python3 -c 'from osp_mission.wall_bearings import WallBearings'; fi
mkdir "$trial"
for f in run world scene.json scene.yaml; do ln -s "$(realpath -e "$base/$f")" "$trial/$f"; done
echo "$revision" > "$trial/policy_revision.txt"
export BENCH_POLICY="$mode" BENCH_RUN="$trial" BENCH_ROOT="$trial"
export ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="file://$suite/2026-11-19_pair_7_103/cyclone.xml"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ROS_HOME="$trial/ros_home" ROS_LOG_DIR="$trial/ros_logs"
unset ROS_STATIC_PEERS ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
python3 -u "$suite/2026-11-19_corridor_box/measure_video.py" >"$trial/observer.log" 2>&1
python3 "$suite/2026-11-19_corridor_box/finish_trial.py" "$trial" "$trial" >>"$trial/observer.log" 2>&1
