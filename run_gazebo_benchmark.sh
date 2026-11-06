#!/usr/bin/env bash
# Run the balanced native Gazebo suite once; each invocation has fresh outputs.
set -eo pipefail
cd /home/peter/simul
source /opt/ros/jazzy/setup.bash
export ROS_LOG_DIR=/home/peter/simul/training_robust/ros_logs
export OPENBLAS_NUM_THREADS=1
colcon build --symlink-install --packages-select osp_mission
source install/setup.bash
osp_benchmark_output="/home/peter/simul/training_robust/gazebo_campaigns/$(date -u +%Y%m%dT%H%M%S_%NZ)"
bash run_gazebo_guarded.sh python3 training_robust/gazebo_benchmark.py \
    --scenarios training_robust/gazebo_runtime_calibration/scenarios \
    --policy training_robust/gazebo_runtime_calibration/policy.json \
    --output "$osp_benchmark_output" --limit 20 --record-video "$@"
