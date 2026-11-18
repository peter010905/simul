#!/usr/bin/env bash
set -euo pipefail
repo="$(cd "$(dirname "$0")/../.." && pwd)"
export OSP_ROBOT_WORKSPACE=/home/ubuntu/flow_v3_r32_print_ws
exec "$repo/robot.sh" "${@:-status}"
