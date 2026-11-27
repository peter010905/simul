# v1.5.2.69 thin obstacles (experimental candidate, not production)

Parent: v1.5.2.68 contact escape (on .67.1). Files: `thin_obstacles.py` (new), `mission.py`.

## Why: avoid the stuck positions instead of only escaping them
In the three stuck runs of 2026-11-27/28 the goal itself was in open space (1.15-1.32 m clear).
On the way, the robot drove straight at a single leg and stopped 0.23-0.29 m from it (blob centre),
not under a desk.
- Legs 2-4 cm across are hit by only some beams. The global obstacle layer's scan source marks and
  clears them in turn, so a path could run right past a leg.
- The physical controller (Regulated Pure Pursuit) follows the path without avoiding obstacles.
- The raw-scan collision monitor then stopped the robot against the leg.

The hardware Nav2 profile already has a marking-only global source, `mission/known_obstacles`
(clearing: false). It was published only when `undock` was set, so it stayed empty on the robot.

## Rule
Every tick, the occupied components of the SLAM map no larger than 0.30 m (legs; boxes are 0.46 m
or more) are published on `mission/known_obstacles` in the map frame. The extraction is redone only
when the map changes. The planner keeps these points and inflates them (0.9 m since .62.1), so paths
stay away from legs even while the scan misses them. The dock points, when undocking, go into the same
cloud in the map frame. The collision monitor and the local costmap are unchanged.

## Install
`python3 install.py <.68 workspace> <new workspace>` then `colcon build`.
Physical drive: none at install. Board default unchanged.
