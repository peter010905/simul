"""Observe both applied stressors; a launched relay is not input verification."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import time


def gate(report, expected):
    scan=report.get('scan',{});wheel=report.get('wheel',{})
    return bool(scan.get('profile')==expected and scan.get('received',0)>=3
        and scan.get('acquisition_stamp_preserved') is True and scan.get('ground_truth_input') is False
        and wheel.get('transfer_profile')==expected and wheel.get('accepted',0)>=3
        and wheel.get('ground_truth_input') is False)


def probe(domain, output):
    command='source /opt/ros/jazzy/setup.bash\nexec '+shlex.join([
        '/usr/bin/python3',str(Path(__file__).resolve()),'--domain',str(domain),'--output',str(output)])
    subprocess.run(['/bin/bash','-c',command],check=True,capture_output=True,text=True,timeout=25)
    return json.loads(output.read_text())


def measure(domain):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    context=rclpy.Context();rclpy.init(context=context,domain_id=domain)
    node=Node('transfer_input_observer',context=context);values={}
    def receive(name, message):
        values[name]=json.loads(message.data)
    for name,topic in [('scan','mission/transfer_status'),('wheel','mission/wheel_twist_status')]:
        node.create_subscription(String,topic,lambda m,n=name:receive(n,m),10)
    deadline=time.monotonic()+15
    try:
        while time.monotonic()<deadline and len(values)<2:rclpy.spin_once(node,timeout_sec=.2)
    finally:
        node.destroy_node();context.shutdown()
    return values


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--domain',type=int,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.write_text(json.dumps(measure(a.domain),indent=2)+'\n')
