# Project maintenance

- Physical robot commands on this Mac use `./robot.sh start` and
  `./robot.sh stop` with the prepared `face_intent_flow_v4r2_ws` in `osp-jazzy`.
  The separate Nov 17 Flow v3 + MPPI candidate uses `./robot.sh start-v3-mppi`
  and `face_intent_v3_sweep_ws` in the same VM.
  The physical board also supports explicit start/stop buttons when launched
  with `--enable-control`. Both use `scripts/robot_session.py` and the same
  readiness checks. Do not rebuild, redeploy, restart the board, or manually
  call `/mission/start` for an ordinary drive request. Execute the prepared
  entry point first; investigate only a reported failure. `check` and `status`
  are read-only. A stop remains in force until the user explicitly requests
  driving or presses the board's start button. Never start for a UI/code test.

- The user authorized ongoing maintenance of the private GitHub repository
  `peter010905/simul`. After completing a project change, run relevant checks,
  review the intended diff, commit it, and push to `main`. No repeated upload
  approval is needed. Do not claim that unattended code review or scheduled
  maintenance exists unless it has actually been configured.
- Update the README and `repository_snapshot/status.json` when reporting a new
  experiment milestone. Distinguish original benchmark results, offline replay,
  and new Gazebo results. Never promote an unverified candidate as production.
- Publish finished simulation videos as private GitHub Release assets using
  `scripts/publish_simulation_videos.py`; commit the generated video index.
  Do not add video binaries, generated maps, ROS logs, or build output to Git.
- Preserve original experiments. Do not edit source in a frozen runtime stage
  while its campaign is running or queued. Create a new staged candidate.
- Run at most two native Gazebo benchmarks concurrently on this host. Preserve
  the existing service memory limits and independent evaluation ground truth.
- Required task outcome: three accurate marker IDs, actual approach within
  1.5 m of each marker, correct map locations, and fast collision-free completion.
  Ground-truth scenario data belongs only to the evaluator, never the policy.
