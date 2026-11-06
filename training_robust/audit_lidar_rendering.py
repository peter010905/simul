"""Native Gazebo sensor diagnostic in a closed, analytically known square."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--extra-sensor', action='store_true')
    parser.add_argument('--software', action='store_true')
    args = parser.parse_args()
    os.environ['GZ_PARTITION'] = f'osp_lidar_audit_{os.getpid()}'
    if args.software:
        os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
    from gz.transport13 import Node
    from gz.msgs10.laserscan_pb2 import LaserScan
    root = Path('training_robust/native_diagnostics/lidar_rendering')
    root.mkdir(parents=True, exist_ok=True)
    name = ('multi' if args.extra_sensor else 'single') + ('_software' if args.software else '_default')

    def lidar(name, samples, low, high, near, far):
        return f'''<model name='{name}'><static>true</static><pose>0 0 .2 0 0 0</pose><link name='link'>
        <sensor name='{name}' type='gpu_lidar'><topic>/{name}</topic><update_rate>5</update_rate>
        <lidar><scan><horizontal><samples>{samples}</samples><resolution>1</resolution><min_angle>{low}</min_angle><max_angle>{high}</max_angle></horizontal>
        <vertical><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></vertical></scan>
        <range><min>{near}</min><max>{far}</max><resolution>.01</resolution></range></lidar></sensor></link></model>'''

    walls = ''
    for i, (x, y, sx, sy) in enumerate([(4,0,.2,8),(-4,0,.2,8),(0,4,8,.2),(0,-4,8,.2)]):
        walls += f'''<model name='wall{i}'><static>true</static><pose>{x} {y} .5 0 0 0</pose><link name='link'>
        <visual name='visual'><geometry><box><size>{sx} {sy} 1</size></box></geometry></visual>
        <collision name='collision'><geometry><box><size>{sx} {sy} 1</size></box></geometry></collision></link></model>'''
    sensors = lidar('main_scan',640,-math.pi,math.pi,.164,12)
    if args.extra_sensor:
        sensors += lidar('short_scan',5,-.087,.087,.025,.2)
    sdf = root/f'{name}.sdf'
    sdf.write_text(f'''<sdf version='1.9'><world name='audit'><physics name='physics' type='ignored'><max_step_size>.001</max_step_size><real_time_factor>1</real_time_factor></physics>
    <plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'/>
    <plugin filename='gz-sim-sensors-system' name='gz::sim::systems::Sensors'><render_engine>ogre2</render_engine></plugin>
    <plugin filename='gz-sim-scene-broadcaster-system' name='gz::sim::systems::SceneBroadcaster'/>{walls}{sensors}</world></sdf>''')
    node = Node()
    received = []
    node.subscribe(LaserScan, '/main_scan', lambda msg: received.append(msg))
    if args.extra_sensor:
        node.subscribe(LaserScan, '/short_scan', lambda msg: None)
    log = (root/f'{name}.log').open('w')
    proc = subprocess.Popen(['gz','sim','-s','-r','--headless-rendering',str(sdf.resolve())], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    start = time.monotonic()
    try:
        while len(received)<20 and time.monotonic()-start<30 and proc.poll() is None:
            time.sleep(.1)
        report = dict(case=name,frames=len(received),wall_seconds=time.monotonic()-start)
        if received:
            m=received[-1];v=np.asarray(m.ranges);a=m.angle_min+np.arange(len(v))*m.angle_step
            expected=3.9/np.maximum(np.abs(np.sin(a)),np.abs(np.cos(a)))
            report.update(beams=len(v),finite=int(np.isfinite(v).sum()),
                          sector_finite=[int(np.isfinite(z).sum()) for z in np.array_split(v,8)],
                          error_quantiles=np.quantile(np.abs(v-expected),[.5,.95]).tolist())
            np.savez_compressed(root/f'{name}.npz',ranges=v,angles=a,expected=expected)
        (root/f'{name}.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(6)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        log.close()


if __name__ == '__main__':
    main()
