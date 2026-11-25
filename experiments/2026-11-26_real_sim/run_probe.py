"""Bounded native sensor test with isolated ROS/Gazebo and exact-process cleanup."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET


def run(workspace, fixture, output, profile=None):
    if output.exists():
        raise FileExistsError(output)
    output.mkdir(parents=True)
    shutil.copyfile(fixture/'scene.yaml', output/'scene.yaml')
    (output/'world').mkdir()
    tree = ET.parse(fixture/'world/osp_studio.sdf')
    for element in tree.iter():
        if element.text and '@TEAM_MAP_ROOT@' in element.text:
            element.text = 'file://'+element.text.replace('@TEAM_MAP_ROOT@', str(fixture))
    tree.write(output/'world/osp_studio.sdf')
    env = dict(os.environ, ROS_DOMAIN_ID='185', ROS_LOCALHOST_ONLY='1',
        ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
        CAMERA_BENCH_WIDTH='640', CAMERA_BENCH_HEIGHT='480', CAMERA_BENCH_FPS='5',
        REAL_SIM_FIXTURE=str(output), REAL_SIM_PROBE=str(output),
        ROS_HOME=str(output/'ros_home'), ROS_LOG_DIR=str(output/'ros_logs'),
        OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', GZ_PARTITION='real_sim_'+output.name)
    for name in ('ROS_STATIC_PEERS', 'ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                 'FASTDDS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI', 'OSP_REAL_SIM_PROFILE'):
        env.pop(name, None)
    if profile:
        env['OSP_REAL_SIM_PROFILE'] = str(profile)
    cyclone = output/'cyclone.xml'
    cyclone.write_text('<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/>'
        '</Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery>'
        '<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>105</MaxAutoParticipantIndex>'
        '<Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
    env['CYCLONEDDS_URI'] = 'file://'+str(cyclone)
    (output/'manifest.json').write_text(json.dumps(dict(workspace=str(workspace), fixture=str(fixture),
        fixture_sha256=hashlib.sha256((fixture/'world/osp_studio.sdf').read_bytes()).hexdigest(),
        profile=str(profile) if profile else None, physical_drive=False, no_mission=True), indent=2)+'\n')
    here = Path(__file__).resolve().parent
    lockpath = Path.home()/'track-recordings/.domain.lock'
    lockpath.parent.mkdir(exist_ok=True)
    with lockpath.open('a') as lock, (output/'launch.log').open('w') as log:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        process = subprocess.Popen(['ros2', 'launch', str(here/'probe.launch.py')], env=env,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        observer = None
        try:
            observer = subprocess.Popen([sys.executable, str(here/'probe.py'), str(output/'sensors.json')], env=env)
            deadline = time.monotonic()+135
            while observer.poll() is None:
                if process.poll() is not None:
                    raise RuntimeError('Sensor launch exited before measurements completed; inspect launch.log')
                if time.monotonic() > deadline:
                    raise TimeoutError('Native sensor probe exceeded 135s')
                time.sleep(.2)
            if observer.returncode:
                raise RuntimeError('Incomplete native sensor probe; inspect sensors.json')
        finally:
            if observer is not None and observer.poll() is None:
                observer.terminate()
                try:
                    observer.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    observer.kill()
                    observer.wait(timeout=3)
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            # ROS launch may place individual nodes in separate process groups.
            # Only our uniquely tagged sensor-probe descendants are eligible.
            import psutil
            leftovers = []
            for proc in psutil.process_iter():
                try:
                    if proc.pid != os.getpid() and proc.environ().get('REAL_SIM_PROBE') == str(output):
                        leftovers.append(proc)
                        proc.terminate()
                except psutil.Error:
                    pass
            _, alive = psutil.wait_procs(leftovers, timeout=3)
            for proc in alive:
                try:
                    proc.kill()
                except psutil.Error:
                    pass
    print('COMPLETED', output, flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('workspace', type=Path)
    p.add_argument('fixture', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--profile', type=Path)
    a = p.parse_args()
    run(a.workspace, a.fixture, a.output, a.profile)
