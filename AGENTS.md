# Project maintenance

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
