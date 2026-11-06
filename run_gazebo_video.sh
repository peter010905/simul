#!/usr/bin/env bash
# Default: 12 obstacles, three markers on the start-opposite obstacle faces.
# Override with --scene-index 0 to repeat the failed undock calibration scene.
set -eo pipefail
cd /home/peter/simul
bash ./run_gazebo_runtime_probe.sh --record-video --scene-index 16 "$@"
