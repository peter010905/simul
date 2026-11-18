"""Recover a disconnected wired VM before an explicitly requested robot start."""
import argparse
import shlex
import subprocess
import sys
import time


def run(command, timeout=15):
    return subprocess.run(command, capture_output=True, text=True, timeout=timeout)


def vm_command(vm, command, timeout=15):
    return run(['multipass', 'exec', vm, '--', 'bash', '-c', command], timeout)


def recover(vm, repo, server):
    ping = 'ping -c 1 -W 1 ' + shlex.quote(server) + ' >/dev/null 2>&1'
    if vm_command(vm, ping).returncode == 0:
        return
    host = run(['ping', '-c', '1', '-W', '1000', server])
    if host.returncode:
        raise RuntimeError('Mac cannot reach robot ' + server + '. Check robot power and wired connection.')
    # A VM power cycle must never interrupt an active robot supervisor or launch.
    guard = """import sys
from pathlib import Path
sys.path.insert(0, str(Path(sys.argv[1])/'scripts'))
from robot_session import read_state, owned_alive, tracked_group_active
s=read_state(Path.home()/'.local/state/osp-robot/domain-4/session.json')
raise SystemExit(1 if owned_alive(s,'supervisor') or tracked_group_active(s) else 0)
"""
    check = vm_command(vm, 'python3 -c ' + shlex.quote(guard) + ' ' + shlex.quote(repo))
    if check.returncode:
        raise RuntimeError('VM connection failed while a robot session may be active. Recovery refused: ' + check.stderr.strip())
    print('Mac 연결 정상 · VM 유선 연결 복구 중', flush=True)
    for action in ('stop', 'start'):
        result = run(['multipass', action, vm], timeout=120)
        if result.returncode:
            raise RuntimeError(result.stderr or result.stdout)
    deadline = time.monotonic() + 20
    while vm_command(vm, ping).returncode:
        if time.monotonic() >= deadline:
            raise RuntimeError('VM wired connection still unavailable after one recovery attempt')
        time.sleep(1)
    # Recreate the read-only ROS subscriptions and explicit control buttons.
    # Starting the board never sends a mission start request.
    board = f'''source /opt/ros/jazzy/setup.bash
source /home/ubuntu/face_intent_flow_v4r2_ws/install/setup.bash
export ROS_DOMAIN_ID=4 RMW_IMPLEMENTATION=rmw_fastrtps_cpp OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export FASTRTPS_DEFAULT_PROFILES_FILE={shlex.quote(repo+'/robot_profiles/wired_flow_v3/fastdds_vm.xml')}
if ! ss -Hlt 'sport = :8767' | grep -q .; then
nohup setsid python3 -u {shlex.quote(repo+'/dashboard/robot_board.py')} --domain 4 --host 0.0.0.0 --port 8767 --enable-control --control-workspace /home/ubuntu/face_intent_flow_v4r2_ws </dev/null >/tmp/osp-board-recovered.log 2>&1 &
fi
'''
    result = vm_command(vm, board)
    if result.returncode:
        raise RuntimeError('Connection recovered but board restart failed: ' + result.stderr)
    print('VM 유선 연결 복구 완료 · 기존 시간·센서 점검 진행', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vm', required=True)
    parser.add_argument('--repo', required=True)
    parser.add_argument('--server', required=True)
    args = parser.parse_args()
    try:
        recover(args.vm, args.repo, args.server)
    except (RuntimeError, OSError, subprocess.TimeoutExpired) as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
