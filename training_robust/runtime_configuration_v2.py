"""Read effective ROS profiles, including global and local collision maps.

Versioned separately because v3 campaign freezes the preceding probe.
"""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]
REQUESTS={'bt_navigator':['default_server_timeout','use_sim_time'],
          'mission':['policy_file','duration','planner'],
          'slam_toolbox':['restamp_tf','use_sim_time'],
          'local_costmap/local_costmap':['plugins','global_frame','rolling_window'],
          'global_costmap/global_costmap':['plugins','global_frame'],
          'controller_server':['FollowPath.use_collision_detection']}


def configuration_gate(report,policy,expected_ack,live_local=True):
    values=report.get('parameters',{})
    wanted={'bt_navigator':{'default_server_timeout':expected_ack,'use_sim_time':True},
            'mission':{'policy_file':str(policy),'duration':600.,'planner':'edge'},
            'slam_toolbox':{'restamp_tf':True,'use_sim_time':True},
            'local_costmap/local_costmap':{'plugins':(['voxel_layer','inflation_layer'] if live_local else ['static_layer','voxel_layer','inflation_layer']),
                'global_frame':'odom','rolling_window':True},
            'global_costmap/global_costmap':{'plugins':['static_layer','obstacle_layer','inflation_layer'],'global_frame':'map'},
            'controller_server':{'FollowPath.use_collision_detection':True}}
    missing=[];mismatches=[]
    for node,params in wanted.items():
        for name,expected in params.items():
            if name not in values.get(node,{}):
                missing.append(f'{node}/{name}')
            elif values[node][name]!=expected:
                mismatches.append(dict(parameter=f'{node}/{name}',expected=expected,observed=values[node][name]))
    return dict(passed=not missing and not mismatches,missing=missing,mismatches=mismatches)


def probe(domain,output):
    command='source /opt/ros/jazzy/setup.bash\nexec '+shlex.join([
        '/usr/bin/python3',str(Path(__file__).resolve()),'--domain',str(domain),'--output',str(output)])
    subprocess.run(['/bin/bash','-c',command],capture_output=True,text=True,check=True,timeout=30)
    return json.loads(output.read_text())


def measure(domain):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rcl_interfaces.srv import GetParameters
    from rcl_interfaces.msg import ParameterType
    context=rclpy.Context()
    rclpy.init(context=context,domain_id=domain)
    node=Node('native_effective_configuration_probe',context=context)
    executor=SingleThreadedExecutor(context=context)
    executor.add_node(node)
    clients={name:node.create_client(GetParameters,f'/{name}/get_parameters') for name in REQUESTS}
    pending={};values={};errors={}
    deadline=time.monotonic()+20
    try:
        while time.monotonic()<deadline and len(values)<len(REQUESTS):
            for name,client in clients.items():
                if name not in pending and client.service_is_ready():
                    request=GetParameters.Request(names=REQUESTS[name]);pending[name]=client.call_async(request)
            executor.spin_once(timeout_sec=.1)
            for name,future in pending.items():
                if name in values or not future.done():
                    continue
                try:
                    fields={}
                    for key,value in zip(REQUESTS[name],future.result().values):
                        if value.type==ParameterType.PARAMETER_BOOL:fields[key]=bool(value.bool_value)
                        elif value.type==ParameterType.PARAMETER_INTEGER:fields[key]=int(value.integer_value)
                        elif value.type==ParameterType.PARAMETER_DOUBLE:fields[key]=float(value.double_value)
                        elif value.type==ParameterType.PARAMETER_STRING:fields[key]=value.string_value
                        elif value.type==ParameterType.PARAMETER_STRING_ARRAY:fields[key]=list(value.string_array_value)
                    values[name]=fields
                except Exception as exc:errors[name]=str(exc)
        return dict(domain=domain,parameters=values,errors=errors,
                    complete=len(values)==len(REQUESTS),source='Read-only ROS GetParameters services')
    finally:
        executor.shutdown();node.destroy_node();rclpy.shutdown(context=context)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain',type=int,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=measure(args.domain)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
