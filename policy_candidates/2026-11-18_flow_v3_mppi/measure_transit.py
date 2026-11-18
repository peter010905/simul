"""Measure the first 50 mission seconds in an isolated, headless Gazebo domain."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.msg import State
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger


def summarize(samples):
    if len(samples) < 2:
        raise RuntimeError('Too few odometry samples to measure motion')
    path = moving = turning = stopped = rolling_turn = missing = 0.
    pauses = 0
    has_moved = False
    pause_duration = 0.
    for a, b in zip(samples, samples[1:]):
        dt = b[0] - a[0]
        if not 0 < dt <= .5:
            missing += max(dt, 0.)
            pause_duration = 0.
            continue
        path += math.dist(a[1:3], b[1:3])
        if abs(a[3]) > .02:
            has_moved = True
            moving += dt
            if abs(a[4]) > .08:
                rolling_turn += dt
        elif abs(a[4]) > .08:
            turning += dt
        else:
            stopped += dt
            if has_moved:
                pause_duration += dt
                if pause_duration-dt < .5 <= pause_duration:
                    pauses += 1
            continue
        pause_duration = 0.
    changes = {}
    if not missing:
        t, _, _, linear, angular = np.asarray(samples).T
        grid = np.arange(t[0], t[-1], .2)
        for label, values in [('linear_accel_m_s2', linear), ('angular_accel_rad_s2', angular)]:
            if len(grid) >= 2:
                accel = np.abs(np.diff(np.interp(grid, t, values))) / .2
                changes[label] = {'mean_abs': float(np.mean(accel)), 'p95_abs': float(np.percentile(accel, 95))}
    return dict(**changes, path_m=path, net_m=math.dist(samples[0][1:3], samples[-1][1:3]),
                moving_s=moving, turning_only_s=turning, stopped_s=stopped,
                rolling_turn_s=rolling_turn, pauses_ge_0_5s=pauses, missing_s=missing,
                observed_s=samples[-1][0]-samples[0][0], odom_samples=len(samples))


# All endpoints are in the same map frame in robust_demo. The mission stays idle.
# Repeat a short straight, a diagonal bend and a longer segment with the same final yaws.
ROUTE = ((.46, .09, .35), (1.29, 1.04, .95), (2.63, 3.43, 1.05))


def fixed_route(node, state, samples, recording, proc):
    navigation = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    deadline = time.monotonic() + 240.
    def until(condition):
        while proc.poll() is None and time.monotonic() < deadline:
            if condition():
                return
            rclpy.spin_once(node, timeout_sec=.1)
        raise RuntimeError('Timed out waiting for the fixed-route navigation')
    received_map = []
    def map_ready(msg):
        if msg.info.width and msg.info.height:
            received_map.append(True)
    subscription = node.create_subscription(OccupancyGrid, 'map', map_ready, 5)
    until(lambda: received_map and state.get('state') == 'idle' and navigation.server_is_ready()
          and all(state.get('nav2_lifecycle', {}).get(name) == State.PRIMARY_STATE_ACTIVE
                  for name in ('controller_server','planner_server','bt_navigator')))
    node.destroy_subscription(subscription)
    recording['active'] = True
    results = []
    for x, y, yaw in ROUTE:
        target = NavigateToPose.Goal()
        target.pose.header.frame_id = 'map'
        target.pose.pose.position.x, target.pose.pose.position.y = x, y
        target.pose.pose.orientation.z = math.sin(yaw / 2)
        target.pose.pose.orientation.w = math.cos(yaw / 2)
        request = navigation.send_goal_async(target)
        until(request.done)
        goal = request.result()
        if not goal.accepted:
            raise RuntimeError(f'Navigation rejected waypoint {(x, y)}')
        future = goal.get_result_async()
        until(future.done)
        result = future.result()
        results.append(result.status)
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            break
    return dict(waypoint_status=results, all_reached=results == [GoalStatus.STATUS_SUCCEEDED] * len(ROUTE),
                **summarize(samples))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--fixed-route', action='store_true', help='Drive three identical Nav2 waypoints in robust_demo')
    parser.add_argument('--scenario', type=Path, help='Override the Gazebo world for an exploration test')
    parser.add_argument('--repo', type=Path, default=Path.home()/'simul')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') not in ('184', '185') or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('Set a dedicated local simulation domain (184 or 185)')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    args_ros = ['ros2','launch','osp_mission','simulation.launch.py','headless:=true','rviz:=false',
                'planner:=edge','record_video:=false','duration:=120.0','camera_width:=320',
                'output_dir:='+str(out), 'nav2_params:='+str(args.repo/'robot_profiles/wired_flow_v3/nav2_hardware_profile.yaml'),
                'policy_file:='+str(args.repo/'policy_releases/2026-11-13_face_intent_v2/policy.json')]
    if args.scenario:
        if args.fixed_route:
            parser.error('The fixed waypoints are defined only in robust_demo')
        args_ros.append('scenario:='+str(args.scenario.resolve()))
    rclpy.init()
    node = rclpy.create_node('transit_measurement')
    state, samples = {}, []
    capturing_fixed_route = {'active': False}
    def status(msg):
        state.clear()
        state.update(json.loads(msg.data))
    def odom(msg):
        if capturing_fixed_route['active'] or state.get('state') == 'running':
            t, p, v = msg.header.stamp, msg.pose.pose.position, msg.twist.twist
            samples.append((t.sec + t.nanosec*1e-9, p.x, p.y, v.linear.x, v.angular.z))
    node.create_subscription(String, 'mission/status', status, 10)
    node.create_subscription(Odometry, 'odom', odom, qos_profile_sensor_data)
    start = node.create_client(Trigger, 'mission/start')
    with (out/'console.log').open('w') as stream:
        proc = subprocess.Popen(args_ros, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            if args.fixed_route:
                result = fixed_route(node, state, samples, capturing_fixed_route, proc)
                (out/'motion.json').write_text(json.dumps(result, indent=2)+'\n')
                print(json.dumps(result), flush=True)
                return
            deadline = time.monotonic() + 300.
            pending, started, retry_after = None, False, 0.
            while time.monotonic() < deadline and proc.poll() is None:
                rclpy.spin_once(node, timeout_sec=.1)
                if pending is not None and pending.done():
                    started, pending = pending.result().success, None
                    retry_after = time.monotonic() + 2.
                if not started and not pending and state.get('state') == 'idle' and start.service_is_ready() and time.monotonic() > retry_after:
                    pending = start.call_async(Trigger.Request())
                if started and (state.get('elapsed_seconds', 0.) >= 50. or state.get('state') == 'done'):
                    break
            else:
                raise RuntimeError(f'Mission did not reach measurement horizon: {state}')
            result = summarize(samples)
            result.update(mission=state)
            (out/'motion.json').write_text(json.dumps(result, indent=2)+'\n')
            print(json.dumps({k:v for k,v in result.items() if k != 'mission'}), flush=True)
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=10.)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
