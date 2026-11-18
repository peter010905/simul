#!/usr/bin/env bash
# Mac entry point for the already-built TurtleBot4 Flow v4 runtime.
set -euo pipefail
action="${1:-status}"
if (($#)); then shift; fi
default_workspace=/home/ubuntu/face_intent_flow_v4r2_ws
case "$action" in
  start-v3-mppi|dry-run-v3-mppi)
    action="${action%-v3-mppi}"
    default_workspace=/home/ubuntu/face_intent_v3_sweep_ws ;;
  start-r32|dry-run-r32)
    action="${action%-r32}"
    default_workspace=/home/ubuntu/r32_drive_original18_ws ;;
  start|stop|check|status|dry-run) ;;
  -h|--help)
    cat <<'HELP'
Usage: ./robot.sh {start|start-v3-mppi|start-r32|stop|check|status|dry-run|dry-run-v3-mppi|dry-run-r32} [--duration SECONDS]
  start    Check clocks/sensors, launch the prepared runtime, then start when ready.
  start-v3-mppi  Run the Nov 17 Flow v3 marker policy with the MPPI transit controller.
  start-r32  Run the separate R32 drive with the Original/18 marker policy.
  stop     Cancel motion, terminate this session, and verify stationary odometry.
  check    Read-only readiness check; never launches the policy or moves the robot.
  status   Read-only live state and tracked process status.
  dry-run  Print the command without contacting the VM.
Defaults: osp-jazzy VM, ROS domain 4, prepared face_intent_flow_v4r2_ws.
Override OSP_ROBOT_VM, OSP_ROBOT_REPO, OSP_ROBOT_WORKSPACE,
OSP_ROBOT_DOMAIN, OSP_ROBOT_DDS_PROFILE or OSP_ROBOT_NTP_SERVER if needed.
Start runs in the foreground; Ctrl-C stops the session. Stop also works in another terminal.
HELP
    exit 0 ;;
  *) printf 'Unknown action: %s\n' "$action" >&2; exit 2 ;;
esac
robot_vm="${OSP_ROBOT_VM:-osp-jazzy}"
robot_repo="${OSP_ROBOT_REPO:-/home/ubuntu/simul}"
robot_workspace="${OSP_ROBOT_WORKSPACE:-$default_workspace}"
robot_domain="${OSP_ROBOT_DOMAIN:-4}"
robot_profile="${OSP_ROBOT_DDS_PROFILE:-$robot_repo/robot_profiles/wired_flow_v3/fastdds_vm.xml}"
robot_ntp="${OSP_ROBOT_NTP_SERVER:-192.168.185.3}"
robot_args=(python3 -u "$robot_repo/scripts/robot_session.py" "$action"
  --repo "$robot_repo" --workspace "$robot_workspace" --domain "$robot_domain"
  --ntp-server "$robot_ntp" "$@")
if [[ "$action" == start || "$action" == dry-run ]]; then
  robot_args[3]=start
  robot_args=(bash "$robot_repo/run_policy.sh" --mode robot --workspace "$robot_workspace"
    --policy "$robot_repo/policy_releases/2026-11-13_face_intent_v2/policy.json"
    -- "${robot_args[@]}")
fi
printf -v robot_command 'source /opt/ros/jazzy/setup.bash\nif [[ -f %q ]]; then source %q; fi\n' \
  "$robot_workspace/install/setup.bash" "$robot_workspace/install/setup.bash"
printf -v robot_env 'export ROS_DOMAIN_ID=%q RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=%q OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1\n' \
  "$robot_domain" "$robot_profile"
printf -v robot_exec '%q ' "${robot_args[@]}"
robot_command+="$robot_env"$'exec '"$robot_exec"
if [[ "$action" == dry-run ]]; then
  printf 'VM: %s\n%s\n' "$robot_vm" "$robot_command"
  exit 0
fi
if [[ "$action" == start && "$robot_domain" == 4 ]]; then
  robot_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
  python3 "$robot_script_dir/scripts/robot_vm_recovery.py" --vm "$robot_vm" --repo "$robot_repo" --server "$robot_ntp"
fi
exec multipass exec "$robot_vm" -- bash -lc "$robot_command"
