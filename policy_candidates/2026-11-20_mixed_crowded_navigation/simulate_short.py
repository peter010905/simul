"""Short, isolated Gazebo integration runs. No physical-domain control."""
import argparse,json,math,os,signal,subprocess,time
from pathlib import Path
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rosgraph_msgs.msg import Clock


def run(label,workspace,root):
    env=dict(os.environ)
    assert env.get('ROS_DOMAIN_ID')=='182' and env.get('ROS_LOCALHOST_ONLY')=='1'
    installed=Path(get_package_share_directory('osp_mission')).resolve()
    if str(Path(workspace).resolve())+'/' not in str(installed):
        raise RuntimeError('Source the requested workspace before this comparison')
    output=root/label;output.mkdir(parents=True)
    cmd=['ros2','launch','osp_mission','simulation.launch.py','headless:=true','rviz:=false',
         'planner:=edge','duration:=90.0','start_docked:=false','camera_width:=640',
         'record_video:=false','render_backend:=software','scenario:='+str(root/'scene.yaml'),
         'output_dir:='+str(output),'policy_file:=/home/ubuntu/simul/policy_releases/2026-11-13_face_intent_v2/policy.json']
    # Launch and observer must use the same chosen overlay; caller sources workspace.
    with (output/'launch.log').open('w') as log:
        process=subprocess.Popen(cmd,env=env,stdout=log,stderr=log,start_new_session=True)
    rclpy.init();node=Node('crowded_synthetic_observer')
    status={};samples=[];timeline=[];sim=[0.]
    def on_status(message):
        nonlocal status
        status=json.loads(message.data)
        timeline.append([sim[0],status])
    def on_odom(message):
        p=message.pose.pose.position;t=message.header.stamp.sec+message.header.stamp.nanosec/1e9
        samples.append([t,p.x,p.y,message.twist.twist.linear.x,message.twist.twist.angular.z])
    node.create_subscription(Odometry,'odom',on_odom,qos_profile_sensor_data)
    node.create_subscription(String,'mission/status',on_status,10)
    node.create_subscription(Clock,'clock',lambda m:sim.__setitem__(0,m.clock.sec+m.clock.nanosec/1e9),qos_profile_sensor_data)
    start=node.create_client(Trigger,'mission/start');stop=node.create_client(Trigger,'mission/stop')
    future=None;next_try=0.;started=None;begin=time.monotonic();error=None
    try:
        while time.monotonic()-begin<300:
            rclpy.spin_once(node,timeout_sec=.1)
            if process.poll() is not None:raise RuntimeError('Launch exited '+str(process.returncode))
            wall=time.monotonic()
            if started is None and start.service_is_ready() and wall>=next_try:
                if future is None:
                    future=start.call_async(Trigger.Request());next_try=wall+1.
                elif future.done():
                    response=future.result();future=None
                    if response.success:
                        started=sim[0];samples.clear();timeline.clear()
                        print(label,'started at',started,flush=True)
            if started is not None and status.get('state')=='done':break
            if started is not None and sim[0]-started>115:raise RuntimeError('Mission save exceeded bounded sim time')
        else:raise TimeoutError('Simulation run exceeded 300 wall seconds')
    except Exception as exc:error=repr(exc)
    finally:
        if status.get('state')=='running' and stop.service_is_ready():
            future=stop.call_async(Trigger.Request())
            limit=time.monotonic()+4
            while not future.done() and time.monotonic()<limit:rclpy.spin_once(node,timeout_sec=.1)
        if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    distance=sum(math.dist(a[1:3],b[1:3]) for a,b in zip(samples,samples[1:]))
    windows=list(zip(samples,samples[1:]))
    stopped=sum(b[0]-a[0] for a,b in windows if abs(a[3])<.015 and abs(a[4])<.06 and 0<b[0]-a[0]<1)
    report=dict(label=label,workspace=workspace,error=error,mission_started_sim_s=started,
                end_sim_s=sim[0],odom_samples=len(samples),path_length_m=distance,
                stopped_s=stopped,final_status=status)
    (output/'measurements.json').write_text(json.dumps(report,indent=2))
    (output/'trajectory.json').write_text(json.dumps(samples))
    (output/'status.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in timeline))
    print(json.dumps(report),flush=True)
    if error:raise RuntimeError(error)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('label');p.add_argument('workspace');p.add_argument('root',type=Path)
    a=p.parse_args();run(a.label,a.workspace,a.root)
