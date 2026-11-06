"""Start one supervised, non-recurring experiment independently of tool sessions.

The service has the same memory limits as run_gazebo_guarded.sh. Results remain
owned by the benchmark; service startup is never interpreted as mission success.
"""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--unit', required=True)
    parser.add_argument('--workspace', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if not re.fullmatch(r'osp-[a-z0-9-]+', args.unit):
        parser.error('Use an osp- service name containing only lowercase letters, digits and hyphens')
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('A command is required after --')
    workspace = args.workspace.resolve()
    output = args.output.resolve()
    if not (workspace/'install/setup.bash').is_file():
        parser.error('Workspace must already be built')
    output.mkdir(parents=True, exist_ok=True)
    script = output/'service_run.sh'
    record_file = output/'service.json'
    if script.exists() or record_file.exists():
        parser.error('Service metadata already exists; preserve the previous run')
    script.write_text('#!/usr/bin/env bash\nset -eo pipefail\n'
                      'source /opt/ros/jazzy/setup.bash\n'
                      f'source {shlex.quote(str(workspace/"install/setup.bash"))}\n'
                      'export OPENBLAS_NUM_THREADS=1\n'
                      f'exec {shlex.join(command)}\n')
    launch = ['systemd-run', '--user', f'--unit={args.unit}',
              f'--property=WorkingDirectory={workspace}',
              '--property=MemoryHigh=5G', '--property=MemoryMax=6G',
              '--property=MemorySwapMax=512M', '--property=KillMode=control-group',
              '--property=TimeoutStopSec=30s', '--property=Restart=no',
              f'--property=StandardOutput=append:{output/"service.log"}',
              f'--property=StandardError=append:{output/"service.log"}',
              '--', '/bin/bash', str(script)]
    record = dict(unit=args.unit+'.service', workspace=str(workspace), command=command,
                  created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  script_sha256=hashlib.sha256(script.read_bytes()).hexdigest(),
                  status='launch_requested', recurring=False)
    record_file.write_text(json.dumps(record, indent=2)+'\n')
    result = subprocess.run(launch, text=True, capture_output=True)
    record.update(launch_returncode=result.returncode, launch_stdout=result.stdout,
                  launch_stderr=result.stderr,
                  status='service_accepted' if result.returncode == 0 else 'launch_failed')
    if result.returncode == 0:
        state = subprocess.run(['systemctl', '--user', 'show', record['unit'],
                                '-p', 'MainPID', '-p', 'ActiveState', '-p', 'SubState'],
                               text=True, capture_output=True, check=True)
        record['observed_service_state'] = state.stdout
    record_file.write_text(json.dumps(record, indent=2)+'\n')
    print(json.dumps(record, indent=2), flush=True)
    raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
