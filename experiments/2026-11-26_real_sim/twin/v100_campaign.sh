#!/bin/bash
# Simulation-only campaign on the shared V100 server. Each episode runs in its own
# container with no network (loopback only), so ROS domain 174 is isolated per episode.
# Usage: v100_campaign.sh <queue.tsv: seed<TAB>label<TAB>workspace> [parallel] [cpus per episode]
# Paths inside the container match the VM (/home/ubuntu/...), mounted from ~/osp-twin/home.
# The server is shared: --cpu-shares 256 yields CPU to other users under contention.
# Software rendering on the Xeon Gold 5120 runs one episode at about 0.35x real time with
# 10-12 cores; 5 cores gave 0.08x. GPU (EGL) rendering in the container stopped the sim clock.
set -uo pipefail
queue=$(realpath "$1"); parallel=${2:-4}; cpus=${3:-12}
home=$HOME/osp-twin/home
mkdir -p "$home/classroom-twin/.v100"
run_one() {
  IFS=$'\t' read -r seed label ws <<<"$1"
  name="trial-track-rand-$seed-$label"
  out=/home/ubuntu/classroom-twin/$name
  [ -e "$home/classroom-twin/$name" ] && return 0
  docker run --rm --network none --cpus "$cpus" --cpu-shares 256 --shm-size 1g -u 1000:1000 \
    -e HOME=/home/ubuntu -v "$home":/home/ubuntu --name "osp-$seed-$label-$$" osp-twin:jazzy \
    bash -c "bash /home/ubuntu/classroom-twin/code/twin/trial.sh /home/ubuntu/classroom-twin/random-room/seed-$seed $ws $out 174" \
    > "$home/classroom-twin/$name.log" 2>&1
  echo "$(date +%T) seed $seed $label exit $?" >> "$home/classroom-twin/.v100/progress.log"
}
export -f run_one; export home cpus
grep -v '^$' "$queue" | xargs -d '\n' -P "$parallel" -I{} bash -c 'run_one "$@"' _ {}
echo "$(date +%T) campaign finished" >> "$home/classroom-twin/.v100/progress.log"
