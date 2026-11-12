"""Capture live process/thread evidence after native navigation errors; never send ROS commands."""
import argparse
import json
from pathlib import Path
import subprocess
import time

from continual_native_curriculum import save
from observe_spatial_campaign import native_pid


TRIGGERS=('CRITICAL FAILURE: SERVER','Failed to activate global_costmap',
          'SLAM startup readiness failed; preserving diagnostics')


def read(path):
    try:return path.read_text(errors='replace').strip()
    except OSError as exc:return f'unavailable:{type(exc).__name__}'


def descendants(parent,table):
    result={parent}
    while True:
        more={pid for pid,ppid in table.items() if ppid in result}
        if more<=result:return result
        result|=more


def snapshot(pid,output,trigger,proc=Path('/proc')):
    # Revalidate identity before collecting anything from a reused PID.
    argv=read(proc/str(pid)/'cmdline').split('\0')
    if not (any(a.endswith('/gazebo_benchmark.py') for a in argv) and str(output) in argv):
        return dict(verified_native_pid=False,trigger=trigger,reason='native_handle_ended_or_changed')
    table={}
    for path in proc.iterdir():
        if not path.name.isdigit():continue
        stat=read(path/'stat').rpartition(')')[2].split()
        if len(stat)>1:
            try:table[int(path.name)]=int(stat[1])
            except ValueError:pass
    processes=[]
    for child in sorted(descendants(pid,table)):
        directory=proc/str(child);command=read(directory/'cmdline').replace('\0',' ')
        # Keep all process summaries; detailed threads only for Nav2's failing path.
        detail=any(name in command for name in ['collision_monitor','lifecycle_manager','slam_toolbox'])
        status={line.split(':',1)[0]:line.split(':',1)[1].strip()
            for line in read(directory/'status').splitlines() if ':' in line}
        row=dict(pid=child,command=command,status={k:status.get(k) for k in
            ['State','PPid','Threads','VmRSS','voluntary_ctxt_switches','nonvoluntary_ctxt_switches']})
        if detail:
            row['threads']=[dict(tid=int(task.name),name=read(task/'comm'),
                wait_channel=read(task/'wchan'),kernel_stack=read(task/'stack'),stat=read(task/'stat'))
                for task in sorted((directory/'task').glob('[0-9]*'))]
        processes.append(row)
    environment=read(proc/str(pid)/'environ').split('\0')
    selected={entry.split('=',1)[0]:entry.split('=',1)[1] for entry in environment
        if '=' in entry and entry.split('=',1)[0] in ['ROS_DOMAIN_ID','RMW_IMPLEMENTATION',
            'FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','FASTDDS_BUILTIN_TRANSPORTS']}
    cgroup=read(proc/str(pid)/'cgroup').split('0::',1)[-1]
    stats={name:read(Path('/sys/fs/cgroup')/cgroup.lstrip('/')/name)
        for name in ['memory.events','memory.current','memory.peak','cpu.stat']}
    return dict(verified_native_pid=True,native_pid=pid,output=str(output),trigger=trigger,
        captured_wall_epoch_s=time.time(),selected_transport_environment=selected,
        cgroup_stats=stats,processes=processes,
        limit='Wait channels and kernel stacks can narrow a stall;they do not prove a user-space deadlock')


def observe(roots):
    offsets={};captures={};started=time.monotonic()
    while time.monotonic()-started<86400:
        active=False
        for root in roots:
            state=json.loads((root/'state.json').read_text())
            active|=state['status'].startswith('running')
            for job in state['jobs']:
                if job['status']!='running':continue
                active=True;output=Path(job['output'])
                try:pid=native_pid(job['unit'],output)
                except subprocess.TimeoutExpired:continue  # Retry the same handle; never restart a native run.
                if pid is None:continue
                log=output/'episodes'/str(job['scenario']['seed'])/'launch.log'
                if not log.exists():continue
                key=str(output)
                with log.open(errors='replace') as stream:
                    stream.seek(offsets.get(key,0));new=stream.read();offsets[key]=stream.tell()
                hits=[line for line in new.splitlines() if any(word in line for word in TRIGGERS)]
                if hits and key not in captures:captures[key]=dict(start=time.monotonic(),trigger=hits[0],steps=set())
                capture=captures.get(key)
                if capture is None:continue
                for delay in [0,5,20]:
                    if delay in capture['steps'] or time.monotonic()-capture['start']<delay:continue
                    report=snapshot(pid,output,capture['trigger'])
                    destination=output/'runtime_failure_capture';destination.mkdir(exist_ok=True)
                    save(destination/f'at_{delay:02d}_wall_seconds.json',report)
                    capture['steps'].add(delay)
        if not active:return
        time.sleep(2)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('roots',type=Path,nargs='+');args=parser.parse_args()
    observe([path.resolve() for path in args.roots])
