"""Read actual TB4 platform ROS parameters and sensor/interface observations."""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import time

import runtime_configuration_v2 as reader

EXPECTED={
    'diffdrive_controller':{'wheel_separation':.233,'wheel_radius':.03575,
        'left_wheel_radius_multiplier':1.,'right_wheel_radius_multiplier':1.,'wheel_separation_multiplier':1.,
        'enable_odom_tf':True,'open_loop':False,'publish_rate':20.},
    'slam_toolbox':{'angle_variance_penalty':1.,'minimum_angle_penalty':.9,'restamp_tf':True,'use_sim_time':True},
    'bt_navigator':{'default_server_timeout':1000,'use_sim_time':True},
    'mission':{'duration':600.,'planner':'edge','require_platform_contract':True},
    'controller_server':{'FollowPath.plugin':'nav2_mppi_controller::MPPIController','enable_stamped_cmd_vel':True},
    'collision_monitor':{'cmd_vel_out_topic':'cmd_vel','enable_stamped_cmd_vel':True},
    'local_costmap/local_costmap':{'plugins':['static_layer','voxel_layer','inflation_layer'],'global_frame':'odom','rolling_window':True},
    'global_costmap/global_costmap':{'plugins':['static_layer','obstacle_layer','inflation_layer'],'global_frame':'map'}}


def configuration_gate(report,policy,expected_ack=1000,live_local=True,expected_duration=600.):
    wanted={node:dict(values) for node,values in EXPECTED.items()};wanted['mission']['policy_file']=str(policy);wanted['mission']['duration']=expected_duration
    observed=report.get('parameters',{});missing=[];mismatches=[]
    for node,values in wanted.items():
        for key,expected in values.items():
            if key not in observed.get(node,{}):missing.append(node+'/'+key)
            elif observed[node][key]!=expected:mismatches.append(dict(parameter=node+'/'+key,expected=expected,observed=observed[node][key]))
    return dict(passed=not missing and not mismatches,missing=missing,mismatches=mismatches)


def probe(domain,output):
    command='source /opt/ros/jazzy/setup.bash\nexec '+shlex.join([
        '/usr/bin/python3',str(Path(__file__).resolve()),'--domain',str(domain),'--output',str(output)])
    subprocess.run(['/bin/bash','-c',command],capture_output=True,text=True,check=True,timeout=50)
    return json.loads(output.read_text())


def motion_gate(report):
    return bool(report.get('platform',{}).get('passed') and report.get('platform',{}).get('ground_truth_input') is False)


def transfer_gate(report,expected):
    scan=report.get('scan',{})
    return bool(motion_gate(report) and scan.get('profile')==expected and scan.get('received',0)>=3
        and scan.get('acquisition_stamp_preserved') is True and scan.get('ground_truth_input') is False)


def measure(domain):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
    reader.REQUESTS={node:list(params) for node,params in EXPECTED.items()};reader.REQUESTS['mission'].append('policy_file')
    report=reader.measure(domain)
    context=rclpy.Context();rclpy.init(context=context,domain_id=domain)
    node=Node('tb4_platform_observer',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    values={}
    def receive(key,message):values[key]=json.loads(message.data)
    for key,topic in [('platform','mission/platform_status'),('scan','mission/transfer_status')]:
        node.create_subscription(String,topic,lambda m,k=key:receive(k,m),10)
    deadline=time.monotonic()+20
    try:
        while time.monotonic()<deadline and (len(values)<2 or not values.get('platform',{}).get('passed')):
            executor.spin_once(timeout_sec=.2)
    finally:
        executor.shutdown();node.destroy_node();context.shutdown()
    return dict(report,**values)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--domain',type=int,required=True)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    args.output.write_text(json.dumps(measure(args.domain),indent=2)+'\n')
