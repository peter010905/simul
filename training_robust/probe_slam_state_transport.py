"""Finite lifecycle RPC transport checks; no Gazebo, scans, or robot commands."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

from continual_native_curriculum import save


def trial(output,domain,transport,number):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from lifecycle_msgs.srv import GetState
    name=f'slam_rpc_probe_{number}';env=dict(os.environ,ROS_DOMAIN_ID=str(domain))
    for key in ('FASTDDS_BUILTIN_TRANSPORTS','FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE'):
        env.pop(key,None)
    if transport=='UDPv4':env['FASTDDS_BUILTIN_TRANSPORTS']=transport
    context=rclpy.Context();rclpy.init(context=context,domain_id=domain)
    node=Node('read_only_slam_rpc_probe',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    client=node.create_client(GetState,f'/{name}/get_state');states=[];pending=None;last=started=time.monotonic()
    with (output/f'{number}_{transport}.log').open('w') as stream:
        process=subprocess.Popen(['/opt/ros/jazzy/lib/slam_toolbox/async_slam_toolbox_node',
            '--ros-args','-r','__node:='+name,'-p','use_sim_time:=false'],env=env,stdout=stream,stderr=stream)
        try:
            while time.monotonic()-started<10 and len(states)<5:
                executor.spin_once(timeout_sec=.05)
                if pending is None and client.service_is_ready():pending=client.call_async(GetState.Request());last=time.monotonic()
                if pending is not None and pending.done():
                    states.append(dict(state=int(pending.result().current_state.id),latency_wall_s=time.monotonic()-last));pending=None
                elif pending is not None and time.monotonic()-last>3:pending=None
                if process.poll() is not None:break
            threads=[]
            for task in Path(f'/proc/{process.pid}/task').glob('*'):
                try:threads.append((task/'comm').read_text().strip())
                except OSError:pass
            report=dict(domain=domain,transport=transport,server_pid=process.pid,states=states,
                five_unconfigured_state_responses=len(states)==5 and all(row['state']==1 for row in states),
                shm_threads=[value for value in threads if 'dds.shm' in value],wall_seconds=time.monotonic()-started)
        finally:
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)
            executor.shutdown();node.destroy_node();rclpy.shutdown(context=context)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--domain',type=int,default=91);parser.add_argument('--trials',type=int,default=3);args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False);results=[]
    for number in range(args.trials*2):
        report=trial(args.output,args.domain,'default' if number%2==0 else 'UDPv4',number);results.append(report)
        print(json.dumps(report),flush=True)
    save(args.output/'summary.json',dict(evidence_type='finite_unconfigured_SLAM_state_RPC_checks_not_native_missions',results=results,
        source='Real isolated SLAM lifecycle services;no simulator or robot commands',
        limit='Lightweight service checks do not reproduce the complete simulation load or establish a cause for an earlier failure'))
