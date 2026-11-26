#!/bin/bash
# Simulation-only policy comparison over random_room.py layouts. Two native Gazebo
# workers at most (domains 174 and 175, the only ones trial.sh accepts).
# Usage: campaign.sh <queue.tsv: seed<TAB>label<TAB>workspace> <fixtures dir> <board root>
# Runs land in <board root>/trial-track-rand-<seed>-<label> so the board lists them.
set -uo pipefail
queue=$(realpath "$1"); fixtures=$(realpath "$2"); out=$(realpath -m "$3")
here=$(dirname "$(realpath "$0")")
state="$out/.campaign"; mkdir -p "$state"
next() {  # atomically claim the next unclaimed queue line
  exec 8>"$state/claim.lock"; flock 8
  local n; n=$(cat "$state/next" 2>/dev/null || echo 0)
  echo $((n + 1)) > "$state/next"
  flock -u 8
  sed -n "$((n + 1))p" "$queue"
}
worker() {
  local domain=$1 line seed label ws run code
  while line=$(next) && [ -n "$line" ]; do
    IFS=$'\t' read -r seed label ws <<<"$line"
    run="$out/trial-track-rand-$seed-$label"
    [ -e "$run" ] && continue
    while :; do
      bash "$here/trial.sh" "$fixtures/seed-$seed" "$ws" "$run" "$domain" > "$run.log" 2>&1
      code=$?
      [ $code -ne 75 ] && break
      rm -rf "$run"; sleep 60   # domain busy with a board run
    done
    echo "$(date +%T) domain $domain seed $seed $label exit $code" >> "$state/progress.log"
  done
}
# WORKERS=1 runs one simulator at a time (two concurrent runs overloaded the 6-CPU VM and
# the observer aborted episodes with action_cancel_unconfirmed).
if [ "${WORKERS:-2}" = 1 ]; then worker 174; else worker 175 & worker 174 & wait; fi
echo "$(date +%T) campaign finished" >> "$state/progress.log"
