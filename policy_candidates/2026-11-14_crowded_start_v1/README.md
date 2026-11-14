# Crowded-start candidate (2026-11-14)

This candidate fixes a reproduced planning failure: a body-clear starting corridor longer than 0.8 m could not reconnect to the normal-clearance exploration region. The previous generator also excluded obstacles near the spawn, leaving this case poorly represented.

The default start connector now searches the observed free-space component until it reaches a normal-clearance region of at least 0.5 m². Every step retains the conservative 0.189 m body radius, cell extent, and 1 cm margin. Unknown/occupied cells remain blocked, cardinal steps prevent corner cutting, and destinations still require 0.32 m clearance. Explicit distance bounds remain supported. Map boundaries are treated as blocked during clearance calculation.

## Evidence

- Complete staged build and 378 runtime tests passed (parent: 373).
- A 55 cm corridor with an exit more than 3 m away reproduces the old empty-candidate failure and produces normal-clearance goals with this change.
- Unknown barriers, occupied barriers, undersized passages, and enclosed pockets remain rejected.
- Four native Gazebo trials are prepared: baseline and candidate on each of two 12-obstacle scenes (55 cm straight start, 60 cm sideways start). Each trial lasts 120 simulation seconds and records video. Independent geometry verifies that the conservative robot body can reach the exit and all three hidden marker approaches. Truth is available only to the simulator/evaluator.
- **Native outcomes are pending. This is a startup regression, not full QR-task qualification or a production release. No hardware validation.**

Frozen workspaces: `training_robust/native90/crowded_start_v1/runtime` and baseline `training_robust/native90/tb4_platform_v5/runtime`. Results: `training_robust/native90/crowded_start_regression_v1` (ignored generated output). The driver freezes source, prepared inputs, and helper hashes. At most two native processes run, retaining the existing 5G/6G/512M memory limits.

Build a fresh workspace:

```bash
python3 policy_candidates/2026-11-14_crowded_start_v1/build.py --workspace /absolute/new/workspace --test
```

Run a new comparison (output directory must not exist):

```bash
python3 training_robust/run_crowded_start_regression.py --output /absolute/new/results --candidate /absolute/candidate/workspace --baseline /absolute/baseline/workspace
```

The separate encoder drift and C-shape SLAM correction jump are unresolved by this change. See `repository_snapshot/tb4_early_odometry_audit.json`. A full 600-second three-marker evaluation remains required after startup validation.
