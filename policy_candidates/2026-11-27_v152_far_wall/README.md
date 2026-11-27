# v1.5.2.62.1 far wall (experimental candidate, not production)

Parent: v1.5.2.62 same view. Requested after physical runs where the robot drove close to walls.

## What the physical runs showed (2026-11-27, 21:02 and 21:07 KST, .62)
The robot's distance to the nearest mapped obstacle was measured during the wall lap, from the
logged decision snapshots.
- Median: 0.64 m and 0.49 m.
- Share of time under 0.6 m: 25% and 67%.

The global costmap inflation ended at 0.45 m (cost_scaling_factor 4.0), so NavFn paths ran about
0.5 m from walls.

## Changes
1. Nav2 global costmap inflation: radius 0.45 -> 0.90 m, cost_scaling_factor 4.0 -> 2.0.
   - Inflated cells stay below the inscribed cost, so aisles between desks stay passable.
   - The local costmap and controller are unchanged.
2. Wall lap stops are 1.2 / 1.5 / 1.8 m from the measured wall, preferring 1.5 m
   (was 0.9 / 1.2 / 1.5, preferring 1.2) (`wall_lap.py`).

Wall-surface look standoffs are unchanged. The coverage ledger accepts a wall look only within
the distance where the smallest marker stays readable (about 0.65 m), so moving those standoffs
out would remove wall coverage.

## Where the Nav2 file is used
`install.py` writes the modified shared profile to `frozen-config/nav2_hardware_profile.yaml`
(read by the Gazebo twin) and to `nav2_override.yaml` (read on hardware). On hardware,
`scripts/robot_session.py` uses `nav2_override.yaml` only when the workspace has one; every
other workspace keeps the shared `robot_profiles/wired_flow_v3/nav2_hardware_profile.yaml`.

## Install
`python3 install.py <.62 workspace> <new workspace> robot_profiles/wired_flow_v3/nav2_hardware_profile.yaml`
then `colcon build`. For a copy-staged Gazebo parent, add `--copy-staged-parent`.
Physical drive: none. Board default unchanged.
