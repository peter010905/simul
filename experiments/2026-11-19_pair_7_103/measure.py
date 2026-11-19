"""Passive truth/bumper observer + simulation-only start/stop and common timing."""
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data as sensor
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
import yaml


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Trial:
    def __init__(self, folder, scene, native):
        self.path, self.native = folder, native
        self.targets = {t['id']: (t['x'], t['y']) for t in scene['targets']}
        self.duration = float(scene.get('duration', 120.))
        self.sim = self.start_sim = self.start_wall = self.pose = None
        self.status = {}
        self.current = set()
        self.registered = {}
        self.crossed = {}
        self.witness = {}
        self.nearest = {}
        self.bump = None
        self.bumps = self.hazard_packets = self.poses = 0
        self.pair_at = None
        self.snapshot = False
        self.node = rclpy.create_node('pair_benchmark_observer')
        prefix = '/osp' if native else '/mission'
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) if native else 10
        self.node.create_subscription(String, prefix+'/status', self.on_status, status_qos)
        if not native:
            self.node.create_subscription(String, '/mission/detector_status', self.on_detector, 10)
        self.node.create_subscription(Clock, '/clock', lambda m: setattr(self, 'sim', seconds(m.clock)), sensor)
        self.node.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image, sensor)
        self.node.create_subscription(TFMessage, '/_internal/sim_ground_truth_pose', self.truth, sensor)
        self.node.create_subscription(HazardDetectionVector, '/hazard_detection', self.hazards, sensor)
        self.start_client = self.node.create_client(Trigger, prefix+'/start')
        self.stop_client = self.node.create_client(Trigger, prefix+'/stop')
        self.file = (folder/'trajectory.csv').open('w')
        self.csv = csv.writer(self.file)
        self.csv.writerow(['elapsed_sim_s','world_x','world_y','registered_ids','bumper'])
        self.events = (folder/'events.jsonl').open('w', buffering=1)

    def image(self, msg):
        if self.snapshot or self.start_sim is None:
            return
        self.snapshot = cv2.imwrite(str(self.path/'start.jpg'), CvBridge().imgmsg_to_cv2(msg, 'bgr8'))

    def on_status(self, msg):
        self.status = json.loads(msg.data)
        if self.native:
            self.set_ids({t['id'] for t in self.status.get('targets', []) if t['confirmed']})

    def on_detector(self, msg):
        self.set_ids(set(json.loads(msg.data).get('resolved_ids', [])))

    def set_ids(self, ids):
        if self.current != ids:
            self.event('ids', ids=sorted(ids))
        self.current = ids
        if self.start_sim is not None:
            for i in ids:
                self.registered.setdefault(i, max(0., self.sim-self.start_sim))

    def event(self, event, **info):
        print(json.dumps(dict(event=event, sim=self.sim, **info)), file=self.events)

    def hazards(self, msg):
        active = any(v.type == HazardDetection.BUMP for v in msg.detections)
        if self.start_sim is not None:
            self.hazard_packets += 1
            if active and not self.bump:
                self.bumps += 1
                self.event('bumper')
        self.bump = active

    def truth(self, msg):
        for tf in msg.transforms:
            if tf.child_frame_id != 'turtlebot4':
                continue
            p = tf.transform.translation
            self.pose = (p.x,p.y)
            if self.start_sim is None:
                return
            dt = seconds(tf.header.stamp)-self.start_sim
            if dt < 0:
                return
            self.poses += 1
            self.csv.writerow([round(dt,4),p.x,p.y,' '.join(map(str, sorted(self.current))),self.bump])
            for i, point in self.targets.items():
                d = math.dist(self.pose, point)
                self.nearest[i] = min(d,self.nearest.get(i,math.inf))
                if d <= 1.5:
                    self.crossed.setdefault(i,dt)
                    if i in self.current:
                        self.witness.setdefault(i,dt)
            if self.targets.keys() <= self.current and self.targets.keys() <= self.witness.keys() and self.pair_at is None:
                self.pair_at = dt

    def prepare_start(self):
        return True

    def run(self):
        pending = None
        deadline = time.monotonic()+235
        next_request = next_log = 0.
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.1)
            if self.sim is None or self.pose is None:
                continue
            ready = self.status.get('state') == ('READY' if self.native else 'idle')
            if time.monotonic() >= next_log:
                print('prestart',self.sim,self.pose,self.status.get('state'),self.status.get('readiness') or self.status.get('readiness_reason'),flush=True)
                next_log = time.monotonic()+12
            if pending is not None and pending.done():
                reply = pending.result()
                pending = None
                if reply.success:
                    self.start_sim = self.sim
                    self.start_wall = time.monotonic()
                    self.event('start_ack', pose=self.pose, ids=sorted(self.current))
                    self.set_ids(self.current)
                    break
                next_request = time.monotonic()+1.
            if (pending is None and ready and self.bump is False and self.prepare_start() and time.monotonic()>=next_request
                    and self.start_client.service_is_ready()):
                pending = self.start_client.call_async(Trigger.Request())
        if self.start_sim is None:
            raise RuntimeError('Start was not acknowledged')
        while time.monotonic()-self.start_wall < max(1200., self.duration*10):
            rclpy.spin_once(self.node,timeout_sec=.1)
            phase = self.status.get('state')
            if self.pair_at is not None:
                reason = ('both_registered_and_within_1.5m' if len(self.targets)==2 else 'all_registered_and_within_1.5m')
                break
            if self.sim-self.start_sim >= self.duration:
                reason = 'common_time_limit'
                break
            if phase in ('FINISHED','done','saving','FINISHING'):
                reason = self.status.get('termination_reason') or self.status.get('reason') or phase
                break
        else:
            raise RuntimeError('Simulation stopped advancing')
        report = dict(policy=os.environ['BENCH_POLICY'], reason=reason, elapsed_sim_s=self.sim-self.start_sim,
                      elapsed_wall_s=time.monotonic()-self.start_wall, pair_time_s=self.pair_at,
                      registered_s=self.registered, final_ids=sorted(self.current), first_within_s=self.crossed,
                      registered_and_within_s=self.witness, min_ground_truth_m=self.nearest, bumper_events=self.bumps,
                      hazard_packets=self.hazard_packets, truth_samples=self.poses,
                      last_policy_status=self.status)
        if not self.poses or not self.hazard_packets:
            raise RuntimeError('Missing ground truth or bumper observations')
        return report


def main(trial_type=Trial, launch_path=None):
    root,folder = map(Path,(os.environ['BENCH_ROOT'],os.environ['BENCH_RUN']))
    if os.environ['ROS_DOMAIN_ID'] not in ('173','174','175') or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Simulation-only domain and loopback required')
    rclpy.init()
    trial = trial_type(folder, yaml.safe_load((root/'scene.yaml').read_text()), os.environ['BENCH_POLICY']=='r32')
    with (folder/'launch.log').open('w') as log:
        child = subprocess.Popen(['ros2','launch',str(launch_path or root/'pair.launch.py')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            report = trial.run()
            report['world_sha256'] = hashlib.sha256((root/'world/osp_studio.sdf').read_bytes()).hexdigest()
            (folder/'metrics.json').write_text(json.dumps(report,indent=2))
            print({k:v for k,v in report.items() if k!='last_policy_status'},flush=True)
        finally:
            if trial.start_sim is not None and trial.stop_client.service_is_ready():
                fut=trial.stop_client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(trial.node,fut,timeout_sec=3.)
            trial.file.close()
            trial.events.close()
            try:
                os.killpg(child.pid,signal.SIGINT)
                child.wait(timeout=15)
            except (ProcessLookupError,subprocess.TimeoutExpired):
                pass
            try:
                os.killpg(child.pid,signal.SIGKILL)
            except ProcessLookupError:
                pass
            trial.node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
