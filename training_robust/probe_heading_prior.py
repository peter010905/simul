"""Read the SLAM heading prior through ROS GetParameters, independently of policy."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess

from continual_native_curriculum import save


EXPECTED={'angle_variance_penalty':.05,'minimum_angle_penalty':.5}


def gate(report):
    values=report.get('parameters',{}).get('slam_toolbox',{})
    return bool(report.get('complete') and all(values.get(k)==v for k,v in EXPECTED.items()))


def probe(domain,output):
    command='source /opt/ros/jazzy/setup.bash\nexec '+shlex.join([
        '/usr/bin/python3',str(Path(__file__).resolve()),'--domain',str(domain),'--output',str(output)])
    subprocess.run(['/bin/bash','-c',command],capture_output=True,text=True,check=True,timeout=30)
    return json.loads(output.read_text())


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain',type=int,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    # Only this fresh probe process changes the request list; the frozen helper file is untouched.
    import runtime_configuration_v2 as reader
    reader.REQUESTS={'slam_toolbox':list(EXPECTED)}
    report=reader.measure(args.domain);report['heading_prior_gate_passed']=gate(report)
    save(args.output,report)
