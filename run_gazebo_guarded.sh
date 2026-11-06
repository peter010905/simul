#!/usr/bin/env bash
# Bound the entire experiment's memory, including all Gazebo/ROS descendants.
set -eo pipefail
exec systemd-run --user --scope --quiet \
    --unit="osp-gazebo-${BASHPID}" \
    -p MemoryHigh=5G -p MemoryMax=6G -p MemorySwapMax=512M \
    "$@"
