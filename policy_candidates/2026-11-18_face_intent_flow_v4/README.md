# Flow v4 physical navigation and recovery

The active revision preserves all eligible untried goals when every goal shares the same departure. It lets the original policy rank them. Generic startup scanning is deferred until planning finds no target. Nav2 counts a 0.35 rad turn as progress; the mission allows recent measured improvements in heading error to extend the translation watchdog only up to 20 s. The original v4 measurements below predate this correction. The corrected baseline passed 396 tests and a 0.35 m closed-loop Gazebo start. [Paired replay and separate R32 version](../2026-11-18_r32_drive_original18/README.md).

For each exploration NavigateToPose goal the mission selects the installed Jazzy
`navigate_w_replanning_time.xml`. It replans at 1 Hz and follows the path. A
controller failure is returned to the mission. Dedicated marker observations
still use the separate Spin action.

At a stall or a controller patience, progress, or no-valid-control error, the
mission remembers the failure in odometry coordinates. Exploration candidates
carry the first route point at least 40 cm from the start. For 60 s it excludes
departures within 45° of the blocked direction. After substantial travel along
the old route it estimates the failed direction from the remaining destination.
The local history clears after 45 cm net travel. The original goal cooldown also
applies. Planner errors exclude the failed destination without requesting a retreat.

The second local failure (or an exhausted set of candidates after the first) requests a rear scan check for up to 1.5 s. A clear
transformed rear sweep, scan age at most 0.5 s, a ready Nav2 BackUp server and
the existing cooldowns permit a single 20 cm retreat at 0.08 m/s at this
location. After a third failure, exploration waits 15 s before the next probe.
Bumper recovery remains a separate trigger. Inspect
`/mission/status.navigation_flow.local_recovery` for the failure count, last
cause and whether the mission dispatched a local retreat.

Build in a fresh workspace with isolated ROS domain 184:

```sh
ROS_DOMAIN_ID=184 ROS_LOCALHOST_ONLY=1 python3 policy_candidates/2026-11-18_face_intent_flow_v4/build.py --workspace /home/ubuntu/face_intent_flow_v4r2_ws --test
```

The 394 Jazzy runtime tests include the actual mission goal, result and retreat
paths. A separate local-only Nav2 navigator experiment injected FollowPath
error 104 and observed exactly one planning call, one control call and an
aborted navigation result carrying code 104. No wheel commands were issued.

The v3 physical baseline is run `20261118T025452-9e81af1edd`. Over its late
98 seconds odometry changed by 4 mm net, and all 11 internal 90° recovery spins
aborted. Rear sweep checks on recorded scans passed 248 of 763 times. In a
pairwise replay of the last 10 goal transitions, remembering only the preceding
failed departure excluded the historically chosen next goal in 8 cases.
Between 5 and 57 original candidates remained in each decision. These are
exploration-grid routes. Nav2 computes its own execution path. Escape, new marker
detections and completion time require a new closed-loop run.
