"""Capture actual SLAM transport before mission readiness,without ROS commands."""
import argparse
import json
from pathlib import Path
import subprocess
import time

from continual_native_curriculum import save
from observe_spatial_campaign import native_pid
from run_pixel_focus_udp_regression import inspect_slam_transport


def observe(root):
    started={};captured={}
    while True:
        state=json.loads((root/'state.json').read_text());pending=False
        for job in state['jobs']:
            if job['status']=='done':continue
            pending=True
            if job['status']!='running':continue
            number=str(job['number']);output=Path(job['output'])
            try:pid=native_pid(job['unit'],output)
            except subprocess.TimeoutExpired:continue
            if pid is None:continue
            report=inspect_slam_transport(pid)
            if not report.get('server_pid'):continue
            key=(number,report['server_pid']);started.setdefault(key,time.monotonic());captured.setdefault(key,set())
            for delay in (0,5,20):
                if delay in captured[key] or time.monotonic()-started[key]<delay:continue
                report.update(elapsed_wall_s=time.monotonic()-started[key],mission_start_proven=False,
                    note='Actual process transport only;this does not establish readiness or mission success')
                folder=output/'slam_transport_capture';folder.mkdir(exist_ok=True)
                save(folder/f'{report["server_pid"]}_at_{delay:02d}s.json',report);captured[key].add(delay)
        if not pending or (captured and all(len(delays)==3 for delays in captured.values())):return
        time.sleep(2)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('root',type=Path);args=parser.parse_args()
    observe(args.root.resolve())
