#!/bin/bash
# Native isolated Gazebo episode. Input is a frozen fixture and a simulation-only workspace.
set -eo pipefail
fixture=$(realpath "$1")
ws=$(realpath "$2")
output=$(realpath -m "$3")
team=${TEAM_REPO:-$HOME/team-official-fixed1122}
test -f "$ws/real_sim_profile.json"
test -f "$fixture/reference.json"
domain=${4:-174}
case "$domain" in 174|175) ;; *) exit 2;; esac
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/classroom-twin-domain-$domain.lock"
flock -n 9 || { echo "Simulation domain $domain is already running" >&2; exit 75; }
test ! -e "$output"
mkdir -p "$output"
mark_exit() {
  code=$?
  [[ -n "${follower_pid:-}" ]] && kill "$follower_pid" 2>/dev/null
  if [[ ! -s "$output/metrics.json" ]]; then
    printf '{"observer_exit":%d}\n' "$code" > "$output/failure.tmp"
    mv "$output/failure.tmp" "$output/failure.json"
  fi
}
trap mark_exit EXIT
python3 - "$ws" "$fixture" "$output/provenance.json" <<'PYMETA'
import hashlib, json, sys, yaml
from pathlib import Path
ws, fixture, out=map(Path,sys.argv[1:])
proof=ws/'hardware_parity.json'
data=json.loads(proof.read_text()) if proof.exists() else {}
ref=json.loads((fixture/'reference.json').read_text())
settings = hashlib.sha256()
files = set((ws/'src').rglob('*.py')) | set((ws/'src').rglob('*.yaml')) | set((ws/'frozen-config').rglob('*')) | set(ws.glob('*.json'))
for file in sorted(f for f in files if f.is_file() and '__pycache__' not in f.parts):
    settings.update(str(file.relative_to(ws)).encode()+b'\0'+hashlib.sha256(file.read_bytes()).digest())
out.write_text(json.dumps(dict(workspace=ws.name, source_settings_sha256=settings.hexdigest(), policy_run=data.get('run',ref['run']),
    pose_run=ref['run'],time_limit_s=yaml.safe_load((fixture/'scene.yaml').read_text())['duration'], scanner_profile=(ws/'scan_signature.json').is_file(),hardware_parity=proof.is_file(), optical_profile=(ws/'optics.json').is_file(),
    world_sha256=hashlib.sha256((fixture/'world/osp_studio.sdf').read_bytes()).hexdigest(),
    policy_sha256=hashlib.sha256((ws/'frozen-config/policy.json').read_bytes()).hexdigest())))
PYMETA
for f in scene.yaml scene.json world run; do ln -s "$fixture/$f" "$output/$f"; done
source /opt/ros/jazzy/setup.bash
source "$ws/install/setup.bash"
export ROS_DOMAIN_ID="$domain" ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_STATIC_PEERS ROS_DISCOVERY_SERVER FASTDDS_DEFAULT_PROFILES_FILE FASTRTPS_DEFAULT_PROFILES_FILE
export CYCLONEDDS_URI="file://$team/experiments/2026-11-19_pair_7_103/cyclone.xml"
export BENCH_ROOT="$output" BENCH_RUN="$output" BENCH_POLICY=mixed CAMERA_POLICY_WORKSPACE="$ws"
export OSP_REAL_SIM_PROFILE="$ws/real_sim_profile.json" TEAM_RANDOM_EXAM=1 OSP_PUBLIC_MARKER_COUNT=3
export CAMERA_BENCH_WIDTH=250 CAMERA_BENCH_HEIGHT=250 CAMERA_BENCH_FPS=10 CAMERA_VARIANT=twin CAMERA_PROFILE=real_log
export CAMERA_BENCH_HELPERS="$team/tools/team_sim/helpers" CAMERA_BENCH_CARDS="$output/cards" SURVEY_EXPERIMENTS="$team/experiments"
export ROS_LOG_DIR="$output/ros_logs" ROS_HOME="$output/ros_home" OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
mkdir -p "$CAMERA_BENCH_CARDS"
if grep -q '<model name="follower"' "$fixture/world/osp_studio.sdf"; then
  python3 -u "$(dirname "$(realpath "$0")")/follower.py" "$(basename "$fixture")" > "$output/follower.log" 2>&1 &
  follower_pid=$!
fi
python3 -u "$team/tools/team_sim/observer.py"
