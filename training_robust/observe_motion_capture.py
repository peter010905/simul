"""Attach bounded read-only encoder/EKF/IMU captures to verified native handles."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import time

from continual_native_curriculum import save
from observe_spatial_campaign import native_pid

ROOT=Path(__file__).resolve().parents[1]


def observe(root,prefix):
    attached={}
    while True:
        state=json.loads((root/'state.json').read_text())
        for job in state['jobs']:
            key=str(job['number'])
            if job['status']!='running' or key in attached:continue
            output=Path(job['output']);episode=output/'episodes'/str(job['scenario']['seed'])
            try:pid=native_pid(job['unit'],output)
            except subprocess.TimeoutExpired:continue
            if not pid or not episode.exists():continue
            target=episode/'encoder_outlier_capture.csv'
            if target.exists():
                attached[key]=dict(status='existing_capture_preserved',path=str(target));continue
            unit=f'{prefix}-{key}'
            command='source /opt/ros/jazzy/setup.bash\nexec '+shlex.join([
                '/usr/bin/python3',str(ROOT/'training_robust/record_motion_calibration.py'),
                '--domain',str(job['domain']),'--native-pid',str(pid),
                '--native-output',str(output),'--output',str(target)])
            result=subprocess.run(['systemd-run','--user','--unit='+unit,
                '--property=MemoryHigh=192M','--property=MemoryMax=256M','--property=MemorySwapMax=0',
                '--property=Restart=no','--property=StandardOutput=append:'+str(output/'motion_recorder.log'),
                '--property=StandardError=append:'+str(output/'motion_recorder.log'),
                '/bin/bash','-c',command],capture_output=True,text=True)
            if result.returncode:save(output/'motion_recorder_launch_error.json',dict(stderr=result.stderr))
            else:attached[key]=dict(native_pid=pid,unit=unit+'.service',path=str(target))
        save(root/'motion_recorders.json',dict(captures=attached,
            ground_truth_available_to_policy=False,source='Read-only evaluator-side encoder/EKF/IMU and independent truth capture'))
        if all(job['status']=='done' for job in state['jobs']):return
        time.sleep(3)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path);parser.add_argument('--prefix',required=True)
    args=parser.parse_args();observe(args.root.resolve(),args.prefix)
