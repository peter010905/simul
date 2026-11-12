"""Finite read-only local-costmap captures during the native ablation.

Observations do not pause, reset, or command the robot. Failed observations are
reported as such; they never restart a live mission or alter a training score.
"""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import time

from continual_native_curriculum import save
from native_tournament import service_active

ROOT=Path(__file__).resolve().parents[1]


def capture(root):
    observation_file=root/'navigation_probe_state.json'
    observed=json.loads(observation_file.read_text()) if observation_file.exists() else dict(
        source='Read-only ROS costmap, LaserScan and TF; no evaluator ground truth',captures={})
    while True:
        state=json.loads((root/'state.json').read_text())
        if all(j['status']=='done' for j in state['jobs']):
            observed['status']='completed';save(observation_file,observed);return
        for job in state['jobs']:
            if job['status']!='running' or not service_active(job['unit']):continue
            output=Path(job['output'])
            live=output/'episodes'/str(job['scenario']['seed'])/'live_status.json'
            if not live.exists():continue
            status=json.loads(live.read_text()).get('status',{})
            if status.get('state')!='running':continue
            elapsed=status.get('elapsed_seconds',0)
            for threshold in (200,400):
                identifier=f'{job["number"]}:{threshold}'
                if identifier in observed['captures'] or elapsed<threshold:continue
                target=output/f'deadlock_probe_{threshold}.json'
                command='source /opt/ros/jazzy/setup.bash\nexec '+shlex.join([
                    '/usr/bin/python3',str(ROOT/'training_robust/probe_navigation_deadlock.py'),
                    '--domain',str(job['domain']),'--output',str(target),'--seconds','8'])
                row=dict(job=job['number'],threshold_sim_s=threshold,elapsed_before_probe_s=elapsed,
                    output=str(target))
                try:
                    result=subprocess.run(['/bin/bash','-c',command],capture_output=True,text=True,timeout=25)
                    row.update(returncode=result.returncode,complete=bool(target.exists()
                        and json.loads(target.read_text()).get('complete')))
                    if result.returncode:row['observation_error']=result.stderr[-1800:]
                except subprocess.TimeoutExpired:
                    row.update(complete=False,observation_error='Read-only probe timeout; mission preserved')
                observed['captures'][identifier]=row;save(observation_file,observed)
                print(json.dumps(row),flush=True)
        time.sleep(5)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path)
    capture(parser.parse_args().root.resolve())
