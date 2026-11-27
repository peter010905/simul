# v1.5.2.68 contact escape (experimental candidate, not production)

Parent: v1.5.2.67.1 floor recovery (Peter09005; on .67 wall jogs). Files: `contact_escape.py`
(new), `mission.py`.

## Why (physical runs 2026-11-27/28)
The robot stopped next to desk legs in three runs. The nearest LiDAR return was 0.17-0.23 m away,
so the footprint (0.189 m) already overlapped an obstacle point. Three things then blocked at once:
- The policy found no reachable goal, because the start cell was at inscribed cost.
- Nav2 spin was blocked.
- The collision monitor's approach polygon cut every command to zero.

The .59/.67 open-space Nav2 step was sent 4 times (10-12 m free), and every time it stalled behind
the collision monitor.

## Rule
After three empty plans, and only when a LiDAR return lies within 0.32 m of the base centre:
1. Turn in place toward the most open 30 deg sector (measured >= 0.7 m). The robot is round, so a
   turn does not change its footprint. Limits: 0.4 rad/s, 12 s.
2. Creep straight at 0.06 m/s for at most 0.30 m (8 s).
3. Stop at once if a return within +-35 deg ahead is closer than 0.25 m, or if Create 3 reports a
   bump, cliff, stall or wheel drop.

The commands go straight to `cmd_vel` at 10 Hz, outside Nav2 and the collision monitor. The rule
runs at most every 20 s. A sector with no echo counts as only 0.8 m clear, so glass cannot attract
the robot. Create 3 hardware safety stays authoritative. Log lines: `Contact escape: ...` and
`Contact escape finished: <reason>`.

## Install
`python3 install.py <.67.1 workspace> <new workspace>` then `colcon build`.
Physical drive: none at install. Board default unchanged.
