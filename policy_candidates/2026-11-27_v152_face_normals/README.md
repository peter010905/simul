# v1.5.2.66 face normals (experimental candidate, not production)

Parent: v1.5.2.65 wall time (.64 lap resume, .63 unswept sides). Adds the .62.1 far-wall settings
and one new rule.

## Why
In the 2026-11-27 physical run (.65, 640x480), the room was rotated about 30 deg in the SLAM map.
All 9 interior face goals were generic samples (none from the box fitter). A generic sample took
its normal from one of the 4 grid directions, so the viewpoint could be up to 45 deg off the real
face. Wall samples already used the smoothed surface-normal field.

Gazebo, 5 seeds where .62 failed all 5: .62.1 far wall found all three prints in 3 of 4 completed
runs. The fifth run did not start (Nav2 bring-up race).

## Changes
1. `center_wall.py`: interior samples use `survey.wall_normal`, the same smoothed field as walls.
   It falls back to the grid direction where the field is weak or disagrees.
2. `wall_lap.py`: lap stops at 1.2 / 1.5 / 1.8 m from the wall, preferring 1.5 m (from .62.1).
3. Nav2: global costmap inflation 0.45 -> 0.90 m, scaling 4.0 -> 2.0 (from .62.1), written to
   `frozen-config` (Gazebo) and `nav2_override.yaml` (hardware, opt-in in robot_session.py).

## Install
`python3 install.py <.65 workspace> <new workspace> robot_profiles/wired_flow_v3/nav2_hardware_profile.yaml`
then `colcon build`. Physical drive: none at install. Board default unchanged.
