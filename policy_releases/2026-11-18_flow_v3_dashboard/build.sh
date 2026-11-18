#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
pkg=src/osp_mission
c++ -std=c++17 -O3 -ffp-contract=off -fno-fast-math -shared -fPIC "$pkg/osp_mission/raycast.cpp" -o "$pkg/osp_mission/_raycast.so"
g++ -std=c++17 -O1 -I/opt/ros/jazzy/include -I/usr/include/eigen3 "$pkg/tools/export_slam_graph_poses.cpp" \
  -L/opt/ros/jazzy/lib -Wl,-rpath,/opt/ros/jazzy/lib -lkartoSlamToolbox -lboost_serialization -lboost_thread -o "$pkg/tools/export_slam_graph_poses"
colcon build --symlink-install --base-paths src --packages-select osp_mission
source install/setup.bash
ROS_DOMAIN_ID=184 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m pytest -q "$pkg/test"
