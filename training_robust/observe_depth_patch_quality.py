"""Attach bounded read-only depth recorders to real campaign native PIDs."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import time

from continual_native_curriculum import save
from observe_spatial_campaign import native_pid

ROOT=Path(__file__).resolve().parents[1]


def observe(root):
    attached={}
    while True:
        state=json.loads((root/'state.json').read_text())
        for job in state['jobs']:
            if job['status']!='running' or str(job['number']) in attached:continue
            output=Path(job['output']);episode=output/'episodes'/str(job['scenario']['seed'])
            pid=native_pid(job['unit'],output)
            if not pid or not episode.exists():continue
            target=episode/'depth_patch_quality.jsonl'
            if target.exists():
                attached[str(job['number'])]=dict(status='existing_capture_preserved',path=str(target));continue
            unit=f'osp-depth-space6-{job["number"]}'
            command='source /opt/ros/jazzy/setup.bash\nexec '+shlex.join([
                '/usr/bin/python3',str(ROOT/'training_robust/record_depth_patch_quality.py'),
                '--domain',str(job['domain']),'--native-pid',str(pid),
                '--native-output',str(output),'--output',str(target)])
            result=subprocess.run(['systemd-run','--user','--unit='+unit,
                '--property=MemoryMax=256M','--property=Restart=no',
                '--property=StandardOutput=append:'+str(output/'depth_quality_recorder.log'),
                '--property=StandardError=append:'+str(output/'depth_quality_recorder.log'),
                '/bin/bash','-c',command],capture_output=True,text=True)
            if not result.returncode:
                attached[str(job['number'])]=dict(native_pid=pid,unit=unit+'.service',path=str(target))
            else:save(output/'depth_recorder_launch_error.json',dict(stderr=result.stderr))
        save(root/'depth_recorders.json',dict(source='Read-only registered-depth quality diagnostics',
            captures=attached,detector_range_provenance_known=False,ground_truth_subscribed=False))
        if state['status']!='running' and not any(j['status']=='running' for j in state['jobs']):return
        time.sleep(3)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('root',type=Path)
    observe(parser.parse_args().root.resolve())
