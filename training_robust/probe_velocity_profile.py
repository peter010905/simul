"""Read actual calibrated encoder routing and retained SLAM prior."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess

from continual_native_curriculum import save

EXPECTED={'calibrated_wheel_twist':{'linear_scale':.78},
    'state_estimator':{'twist0':'wheel/twist_calibrated'},
    'slam_toolbox':{'angle_variance_penalty':.05,'minimum_angle_penalty':.5}}


def gate(report):
    return bool(report.get('complete') and all(report.get('parameters',{}).get(node,{}).get(key)==value
        for node,values in EXPECTED.items() for key,value in values.items()))


def probe(domain,output):
    command='source /opt/ros/jazzy/setup.bash\nexec '+shlex.join([
        '/usr/bin/python3',str(Path(__file__).resolve()),'--domain',str(domain),'--output',str(output)])
    subprocess.run(['/bin/bash','-c',command],capture_output=True,text=True,check=True,timeout=30)
    return json.loads(output.read_text())


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain',type=int,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    import runtime_configuration_v2 as reader
    reader.REQUESTS={node:list(values) for node,values in EXPECTED.items()}
    report=reader.measure(args.domain);report['velocity_profile_gate_passed']=gate(report);save(args.output,report)
