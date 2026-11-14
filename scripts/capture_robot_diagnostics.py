"""Record a real-robot failure without starting/stopping its mission or driving it.

Run in the robot's sourced ROS 2 Jazzy environment and domain. Camera images are
omitted to keep this short diagnostic portable; CameraInfo and marker events
preserve calibration and detection timing evidence.
"""
import argparse
import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time

TOPICS=['scan','tf','tf_static','odom','map','cmd_vel','cmd_vel_smoothed',
        'oakd/rgb/preview/camera_info','mission/status','mission/marker_observations',
        'mission/marker_candidates','hazard_detection','collision_monitor_state',
        'plan','local_plan','parameter_events']
NODES=['mission','slam_toolbox','controller_server','planner_server',
       'global_costmap/global_costmap','local_costmap/local_costmap','collision_monitor']


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--duration',type=float,default=90.)
    p.add_argument('--namespace',default='')
    p.add_argument('--scan-topic',default='scan')
    p.add_argument('--domain',type=int,default=int(os.environ.get('ROS_DOMAIN_ID','0')))
    a=p.parse_args()
    if not 5<=a.duration<=600:p.error('duration must be 5..600 seconds')
    out=a.output.expanduser().resolve();out.mkdir(parents=True,exist_ok=False)
    env=dict(os.environ,ROS_DOMAIN_ID=str(a.domain))
    prefix='/'+a.namespace.strip('/') if a.namespace.strip('/') else ''
    topics=[prefix+'/'+t for t in TOPICS if t!='scan']
    topics.append(a.scan_topic if a.scan_topic.startswith('/') else prefix+'/'+a.scan_topic)
    report=dict(captured_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        domain=a.domain,namespace=a.namespace,topics=topics,parameters={},robot_commands_sent=False)
    command=['ros2','bag','record','-o',str(out/'rosbag'),'--topics',*topics]
    with (out/'recorder.log').open('w') as log:
        recorder=subprocess.Popen(command,stdout=log,stderr=log,env=env,start_new_session=True)
        deadline=time.monotonic()+a.duration
        try:
            for node in NODES:
                try:
                    result=subprocess.run(['ros2','param','dump',prefix+'/'+node],env=env,
                        capture_output=True,text=True,timeout=min(8,max(1,deadline-time.monotonic())))
                    report['parameters'][node]=dict(returncode=result.returncode,yaml=result.stdout,error=result.stderr)
                except subprocess.TimeoutExpired:
                    report['parameters'][node]=dict(error='parameter_read_timeout')
            while time.monotonic()<deadline and recorder.poll() is None:time.sleep(.25)
        finally:
            if recorder.poll() is None:os.killpg(recorder.pid,signal.SIGINT)
            try:report['recorder_exit']=recorder.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(recorder.pid,signal.SIGTERM);report['recorder_exit']=recorder.wait(timeout=5)
            (out/'diagnostics.json').write_text(json.dumps(report,indent=2)+'\n')
    print(out)
    if report['recorder_exit'] not in (0,-signal.SIGINT):raise SystemExit('Recorder failed; inspect recorder.log')


if __name__=='__main__':main()
