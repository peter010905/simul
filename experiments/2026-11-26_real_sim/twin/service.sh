#!/bin/bash
# Install the comparison UI in the VM user's service manager. Does not launch a robot.
set -eo pipefail
root=$(realpath "$1")
workspace=$(realpath "$2")
host=${3:-192.168.252.2}
test -s "$root/stages.json"
test -f "$root/code/twin/board.py"
unitdir="$HOME/.config/systemd/user"
mkdir -p "$unitdir"
cat > "$unitdir/classroom-twin-web.service" <<UNIT
[Unit]
Description=Classroom Gazebo trajectory comparison
After=network-online.target

[Service]
ExecStart=/usr/bin/python3 "$root/code/twin/board.py" "$root" "$workspace" --host "$host"
WorkingDirectory=$root
Restart=on-failure
RestartSec=5

[Install]
WantedBy=default.target
UNIT
systemctl --user daemon-reload
loginctl enable-linger "$USER"
systemctl --user enable --now classroom-twin-web.service
