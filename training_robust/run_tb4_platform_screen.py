"""One short native platform check; never scores it as a 600s policy benchmark."""
import argparse
import csv
import math
import json
from pathlib import Path
import subprocess
import time

import yaml
from native_tournament import native_processes, service_active
from tb4_platform_curriculum import fresh_scene, digest, save, source_hashes
from tb4_platform_probe import probe, configuration_gate, motion_gate
from run_floor_pose_regression import probe as floor_probe, floor_gate

ROOT=Path(__file__).resolve().parents[1]


def run(workspace,output,domain,unit):
    output.mkdir(parents=True,exist_ok=False)
    scene=fresh_scene(1194900000,8,'hidden_obstacles','c_shape')
    path=output/'scenario.yaml';path.write_text(yaml.safe_dump(scene,sort_keys=False))
    policy=ROOT/'policy_releases/2026-11-13_face_intent_v2/policy.json'
    (output/'policy.json').write_bytes(policy.read_bytes())
    save(output/'manifest.json',dict(environment='gazebo_harmonic',planner='edge',camera_width=250,duration=60.,
        render_backend='software',start_docked=False,approach_radius_m=1.5,protocol_version=2,
        policy_sha256=digest(output/'policy.json'),purpose='60s platform integration only; not policy qualification',
        scenarios=[dict(file=str(path),seed=scene['seed'],sha256=digest(path),mount_profile=scene['mount_profile'],
                       requested_obstacles=8,actual_obstacles=len(scene['obstacles']),arena=scene['arena'])]))
    frozen=source_hashes(workspace);save(output/'queued_source_hashes.json',frozen)
    save(output/'screen_status.json',dict(state='waiting_for_native_slot',hardware_validated=False))
    while len(native_processes())>=2:time.sleep(5)
    if source_hashes(workspace)!=frozen:raise RuntimeError('Queued runtime changed')
    subprocess.run(['/usr/bin/python3',str(ROOT/'training_robust/run_native_service.py'),
        '--unit',unit,'--workspace',str(workspace),'--output',str(output),'--',
        '/usr/bin/env','OSP_TRANSFER_PROFILE=none','OSP_TRANSFER_SEED=1194900007',
        '/usr/bin/python3',str(ROOT/'training_robust/gazebo_benchmark.py'),
        '--output',str(output),'--run-prepared','--domain',str(domain),'--record-video'],check=True)
    status=dict(state='running',configuration_gate=False,platform_gate=False,floor_sensor_gate=False,hardware_validated=False)
    episode=output/'episodes'/str(scene['seed'])
    while service_active(unit+'.service'):
        if (episode/'live_status.json').exists() and not (status['configuration_gate'] and status['platform_gate']):
            try:
                result=probe(domain,output/'runtime_configuration.json')
                # Same comparison with this screen's intentionally shorter timer.
                check=configuration_gate(result,output/'policy.json',expected_duration=60.)
                save(output/'runtime_configuration_check.json',check)
                status.update(configuration_gate=check['passed'],platform_gate=motion_gate(result))
            except (OSError,ValueError,subprocess.SubprocessError) as exc:status['observation_error']=repr(exc)
        if status['platform_gate'] and not status['floor_sensor_gate']:
            try:status['floor_sensor_gate']=floor_gate(floor_probe(domain,output/'floor_probe.json'))
            except (OSError,ValueError,subprocess.SubprocessError) as exc:status['floor_observation_error']=repr(exc)
        save(output/'screen_status.json',status)
        time.sleep(5)
    status['state']='completed'
    report=output/'report.json'
    if report.exists():
        row=json.loads(report.read_text())['rows'][0]['result']
        status.update(reason=row.get('reason'),elapsed_seconds=row.get('elapsed_seconds'),contacts=row.get('collisions'),
                      metrics=row.get('runtime_metrics'))
        trajectory=episode/'ground_truth_trajectory.csv'
        points=list(csv.DictReader(trajectory.open())) if trajectory.exists() else []
        status['independent_path_m']=sum(math.hypot(float(b['world_x'])-float(a['world_x']),float(b['world_y'])-float(a['world_y'])) for a,b in zip(points,points[1:]))
        status['passed']=bool(status['independent_path_m']>.5 and status['configuration_gate'] and status['platform_gate'] and status['floor_sensor_gate']
            and row.get('reason') in ('timeout','all_targets_found') and row.get('collisions')==0)
    else:status['passed']=False
    save(output/'screen_status.json',status)
    print(json.dumps(status),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--workspace',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--domain',type=int,default=163)
    p.add_argument('--unit',default='osp-tb4-platform-screen-v1');a=p.parse_args()
    run(a.workspace.resolve(),a.output.resolve(),a.domain,a.unit)
