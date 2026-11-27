# Mixed v1.5.2.67.1 · Floor Recovery

This opt-in variant uses the .67 wall-jogs workspace. The physical rolling voxel costmap and local Nav2 behaviors use a floor frame with the same horizontal axes and origin as odometry. Its stamped vertical offset follows the base. The box planner can therefore use the grid with its existing odometry XY transform. The collision monitor continues to use the raw laser and Create 3 retains authority over its wheels.

Each navigation action gets a fresh odometry-based progress window. Open-space recovery checks the turning envelope, corridor width and endpoint and ranks feasible headings by steering angle. A recent failed destination stays on cooldown. Normal recovery backs up 10 cm through the existing Nav2 action when the rear scan permits it. A Create 3 reverse limit cancels the ordinary retreat. A new retreat requires hardware release, observed forward displacement, fresh rear clearance and the normal action cooldown. Other backup failures retain their longer cooldown.

On the real board select **Mixed v1.5.2.67.1 · 로컬 장애물 지도·탈출 보정**. The original wall-jogs entry is still available.

## Build and verify on ROS 2 Jazzy

```sh
python3 install.py /path/to/v15267_wall_jogs_ws /path/to/new_workspace
cd /path/to/new_workspace
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select osp_mission
source install/setup.bash
python3 -m pytest -q /path/to/2026-11-27_v152_floor_recovery/test_floor_recovery.py
```

The hardware launch configures both local frame consumers and the floor TF publisher. Gazebo keeps its native ground frame. The progress and escape logic runs on both platforms.
