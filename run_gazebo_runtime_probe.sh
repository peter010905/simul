#!/usr/bin/env bash
set -eo pipefail
cd /home/peter/simul
source /opt/ros/jazzy/setup.bash
export ROS_LOG_DIR=/home/peter/simul/training_robust/ros_logs
colcon build --symlink-install --packages-select osp_mission
source install/setup.bash
bash run_gazebo_guarded.sh python3 training_robust/run_runtime_probe.py "$@"
