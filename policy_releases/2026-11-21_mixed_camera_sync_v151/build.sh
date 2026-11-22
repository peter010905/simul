#!/usr/bin/env bash
set -eo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
exec python3 "$here/build.py" "${1:?Pass a new workspace directory}"
