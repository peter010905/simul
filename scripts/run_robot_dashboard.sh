#!/usr/bin/env bash
# Persistent local dashboard; independent from native training services.
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
task_action="${1:-start}"
task_port="${OSP_DASHBOARD_PORT:-8765}"
case "$task_action" in
  start|restart)
    /usr/bin/python3 - "$task_root" "$task_port" <<'PY'
import pathlib, sys
root = pathlib.Path(sys.argv[1])
port = int(sys.argv[2])
if not 1024 <= port <= 65535:
    raise SystemExit('Port must be 1024–65535')
def quote(value):
    return '"' + str(value).replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%') + '"'
directory = pathlib.Path.home() / '.config/systemd/user'
directory.mkdir(parents=True, exist_ok=True)
(directory / 'osp-robot-dashboard.service').write_text(f'''[Unit]
Description=OSP read-only robot dashboard
After=default.target

[Service]
Type=simple
ExecStart=/usr/bin/python3 -u {quote(root / 'dashboard/server.py')} --root {quote(root)} --port {port}
Restart=on-failure
RestartSec=5
MemoryHigh=512M
MemoryMax=768M
CPUQuota=60%
Nice=10
KillMode=control-group
Environment=OPENBLAS_NUM_THREADS=1
Environment=OMP_NUM_THREADS=1
UMask=0077

[Install]
WantedBy=default.target
''')
PY
    systemctl --user daemon-reload
    systemctl --user enable osp-robot-dashboard.service
    systemctl --user restart osp-robot-dashboard.service
    /usr/bin/python3 - "$task_port" <<'PY'
import sys, time, urllib.request
url = 'http://127.0.0.1:' + sys.argv[1] + '/api/state'
for attempt in range(20):
    try:
        with urllib.request.urlopen(url, timeout=1) as response:
            if response.status == 200:
                break
    except OSError:
        time.sleep(0.5)
else:
    raise SystemExit('Dashboard did not become ready; inspect journalctl --user -u osp-robot-dashboard.service')
PY
    printf 'Robot dashboard: http://127.0.0.1:%s\n' "$task_port"
    ;;
  stop) systemctl --user stop osp-robot-dashboard.service ;;
  status) systemctl --user status osp-robot-dashboard.service --no-pager ;;
  run) exec /usr/bin/python3 "$task_root/dashboard/server.py" --root "$task_root" --port "$task_port" ;;
  *) printf 'Usage: %s [start|restart|stop|status|run]\n' "$0" >&2; exit 2 ;;
esac
