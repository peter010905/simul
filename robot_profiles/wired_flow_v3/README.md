# Saved physical robot profile — Original / 18 pixels

This is the exact navigation and wired transport profile captured after physical
run `20261117T131010-6b8219501f`. `profile.json` records settings and file hashes.
The original run manifest has no Git commit; this capture does not invent one.

The normal `./start_robot.command` or `./robot.sh start` entry point now reads
`nav2_hardware_profile.yaml` and `fastdds_vm.xml` from this versioned directory.
The current launcher uses `/home/ubuntu/face_intent_flow_v4r2_ws` with the same sensor and speed profile. The captured v3 workspace in `profile.json` identifies the reference run. There is no
rebuild or policy/dictionary change at start. The physical board uses the same
session launcher and navigation file. The opt-in `./robot.sh start-v3-mppi`
selects the Nov 17 Flow v3 runtime with the MPPI controller and bounded turning
watchdog. Its isolated Gazebo measurements are in the [candidate report](../../policy_candidates/2026-11-18_flow_v3_mppi/README.md).

- Dictionary: `DICT_ARUCO_ORIGINAL`; minimum edge: 18 px; three readable frames.
- Linear limit: 0.25 m/s; angular limit: 0.5 rad/s; no automatic undock.
- Exploration start allowance: 0.30 m, with normal raw collision checks.
- Optional mid-route wall looks disabled; stalled/failed goals are replanned.
- No bumper contact count limit. Retreat requires rear clearance and Nav2 checks.
  Held contact for 10 s, cliff, stall and wheel drop still terminate the session.
- ROS domain 4; wired VM IP 192.168.185.6; robot NTP 192.168.185.3.

For recovery on the existing VM, copy this directory under
`/home/ubuntu/simul/robot_profiles/wired_flow_v3`. The normal launcher sources the
prepared ROS workspace. `./robot.sh check` validates sensors/clock without moving.
`timesyncd.conf` and `sysctl.conf` are captured OS configuration references; they
are not silently applied on each drive request. Robot-side services are described
in `dashboard/ROBOT_BOARD.md` and versioned under `scripts/systemd`.

The runtime reported three confirmed/approached IDs (0, 45, 68), 265.928 s,
zero bump reports and zero sensor interruptions. Sensor-relative three-frame
witnesses were saved. Independent physical ground truth and map coordinate
accuracy were not evaluated, and final graph coordinate correction was rejected
for insufficient matched evidence. This is one recorded run, not general proof.

Raw camera/ROS/map data is retained locally under the ignored `run_logs/robot/`
directory. The user approved public upload; the archive is on GitHub Release
`policy-logs-robot-2026-11`. Both asset sizes and SHA-256 were verified.

Offline video review found all three confirmed examples were handheld screens.
The runtime completion is not validation of autonomous search for fixed markers.
See `repository_snapshot/physical_run_analysis_20261118.json` for measured motion
time, evidence, coordinate-finalization limits and review timestamps.
