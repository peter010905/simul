# v1.5.2.67 wall jogs (experimental candidate, not production)

Parent: v1.5.2.66 face normals (.65 wall time, .64 lap resume, .63 unswept sides, .62.1 far wall).
Built from three problems seen in the 2026-11-27 physical runs, plus one requested change.

## Changes
1. **Wall-step faces** (`wall_jogs.py`, `center_wall.py`). The team marked four pillars and
   recesses on the long walls of the physical map and asked for them to be face-searched.
   - The measured room outline is simplified (0.2 m tolerance) and the room's long axis is
     estimated from the long edges.
   - A run of short edges (0.2-1.5 m) between two parallel long-wall edges, offset at most
     1.2 m, is a step. Each of its edges becomes a face task (`box_view_scope='wall_jog'`).
   - The ends of the room (across the long axis), chair bands and glass zones are excluded.
   - An unvisited step face within a 4 m route is looked at once, first (reason
     `wall_step_face`).
   - Offline on two physical maps: all 4 marked steps were found (7 faces). On the second map,
     2 extra faces were found where the wall was mapped twice.
2. **Open escape** (`mission.py`, from .59, widened). At 342-409 s the robot stood 0.18 m from a
   desk leg, inside the inscribed cost (99). The planner found no reachable goal and repeated
   "checking safe backup" for 50 s without a backup. Now, after three empty plans, whenever a
   backup would not actually run, the robot drives 0.6 m toward the most open scan direction
   (>= 0.9 m clear), at most every 20 s.
3. **Steady lap progress** (`wall_lap.py`). The lap extension uses the furthest progress
   reached. On 2026-11-27 a noisy stop normal dropped the measured turn from 62% to 50%, and the
   lap ended at half the perimeter.

Not changed: the lap's straight-line mode once chose a stop 3.4 m inside the room (stop 10). The
cause is not yet confirmed.

## Install
`python3 install.py <.66 workspace> <new workspace> robot_profiles/wired_flow_v3/nav2_hardware_profile.yaml`
then `colcon build`. Physical drive: none at install. Board default unchanged.
