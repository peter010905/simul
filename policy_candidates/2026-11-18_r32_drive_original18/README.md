# R32 navigation with the existing Original/18 marker policy

The board offers **기존 정책 구동** and **R32 혼합 구동**. The shell equivalents are
`./robot.sh start` and `./robot.sh start-r32`. Both use `./robot.sh stop`, the
same hardware checks and the same per-domain session lock. Installed workspaces
are `~/face_intent_flow_v4r2_ws` and `~/r32_drive_original18_ws`.

The hybrid calls the pinned reference's real R32 exploration planner for
ordinary goals. It passes the observed SLAM grid, connected Nav2 costmap,
camera-observed faces, recent visits, failed destinations and time remaining.
It can pick frontier, inspection or short right-preferred detour goals.
Target hints and close approach take priority in the original mission. R32
perception, target registry and mission node are never launched. The measured
marker modules (detector, size, journal, focus, directed look, approach and
graph landmark processing) are byte-identical to the original physical build.

NavFn uses known space, MPPI controls forward motion, and the R32 alignment
weight and near-goal heading distance are 3 and 0.25 m. Platform limits stay
at 0.25 m/s and 0.5 rad/s. The local raw-scan collision monitor, safety
supervisor, and separately checked short recovery backup remain active. R32
stationary exploration looks are disabled; target observation is owned by the
existing mission. Both planners publish their decisions on the board.

The navigation module is taken from the authorized local checkout of
yeonslay/turtlebot4-marker-slam at `a40cbdfaf812544eff988e27934823b8501a4f4b`.
Its source is not redistributed here. Build checks its SHA-256 before staging it
in a new local workspace. On the Jazzy VM:

```sh
ROS_DOMAIN_ID=185 ROS_LOCALHOST_ONLY=1 python3 policy_candidates/2026-11-18_r32_drive_original18/build.py \
  --reference-core /path/to/turtlebot4-marker-slam/src/core.py \
  --workspace /home/ubuntu/r32_drive_original18_ws --test
```

The last physical run (before this revision) started its mission 50.35 s after
the request, with the map/TF pipeline becoming available late. Its initial
observation turn was blocked, the first navigation produced only angular
commands and ended with FollowPath error 105. The direction filter removed
all candidates on the following 19 decisions. A later backup was rejected by
Nav2's collision check. Both signatures were reproduced from saved decisions.

On 19 independently replayed post-failure snapshots the original policy
correction retained untried destinations on all 19. R32 returned a detour on
each snapshot when evaluated independently with the contemporaneous costmap;
these are proposals, not 19 executed retreats. In a distinct closed-loop Gazebo
trial, the hybrid dispatched five goals over 50.08 s and accumulated 6.79 m
of odometry with 4.57 m net displacement and zero reported bump contacts.
Both versions also passed separate forward-motion probes. The first
Fast DDS simulation run timed out waiting for the SLAM lifecycle; these
successful simulations used Cyclone DDS in isolated domains. The physical
board continues to use its wired DDS profile. Physical escape and complete
three-marker performance remain to be measured.
