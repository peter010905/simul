#!/bin/bash
# macOS LaunchAgent entry point. A finished or interrupted pair releases the guest lock.
set -eu
space_kib() { df -Pk "$1" | awk 'END {print $4}'; }
if [ "$(space_kib "$HOME")" -lt 6291456 ]; then
    echo "Waiting for Mac storage (6 GiB required)."
    exit 0
fi
exec /usr/local/bin/multipass exec osp-jazzy -- bash -c '
    set -eu
    runs="$HOME/track-recordings"
    mkdir -p "$runs"
    if ! flock -n "$runs/.domain.lock" true; then
        echo "Current pair still owns the simulator."
        exit 0
    fi
    available=$(df -Pk "$runs" | awk "END {print \$4}")
    if [ "$available" -lt 6291456 ]; then
        echo "Waiting for guest storage (6 GiB required)."
        exit 0
    fi
    # run.sh takes the same lock again, atomically, before generating a new layout.
    export TRACK_RECORDER_OVERLAY="$HOME/track-recorder-zstd"
    export TRACK_MIXED_OVERLAY="$HOME/mixed-box-views-v2"
    exec "$HOME/track-continuous-box-v2/2026-11-19_random_track/run.sh" pair
'
