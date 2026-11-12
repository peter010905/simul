"""Native calibration of corrected floors on the same circle/triangle scenes.

The old runs have independently confirmed invalid cliff input. They are kept
as infrastructure diagnostics, never used as a policy-quality baseline.
"""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import time

from continual_native_curriculum import collect, digest, save, source_hashes
from native_tournament import native_processes, service_active

ROOT = Path(__file__).resolve().parents[1]


def floor_gate(report):
    names = ('cliff_front_left','cliff_front_right','cliff_side_left','cliff_side_right')
    return bool(report.get('samples',{}).get('hazard_frames',0)>0
        and not any(int(k.split(':')[0])==2 and count for k,count in report.get('hazards',{}).items())
        and all(report.get('samples',{}).get(name,0)>=3
                and report.get('cliff_scans',{}).get(name,{}).get('finite',0)>0
                and report['cliff_scans'][name]['finite']==report['cliff_scans'][name]['values']
                for name in names))


def probe(domain, output):
    command = 'source /opt/ros/jazzy/setup.bash\nexec '+shlex.join([
        '/usr/bin/python3',str(ROOT/'scripts/probe_floor_sensors.py'),'--domain',str(domain),
        '--seconds','7','--output',str(output)])
    subprocess.run(['/bin/bash','-c',command],capture_output=True,text=True,check=True,timeout=25)
    return json.loads(output.read_text())


def run(parent, workspace, output):
    output.mkdir(parents=True,exist_ok=False)
    original = json.loads((parent/'state.json').read_text())
    if native_processes():
        raise RuntimeError('Drain the known invalid pair before this calibration')
    if any(j['status']=='running' and service_active(j['unit']) for j in original['jobs']):
        raise RuntimeError('Original native units are still running')
    for job in original['jobs']:
        if job['status']=='running':
            collect(job)
            save(Path(job['output'])/'environment_invalidation.json',dict(
                reason='Visual floor translated twice and at wrong height; all four cliff scans infinite',
                early_stop='Clean mission/stop, original evaluation/log/video preserved',
                policy_training_sample_valid=False))
    original.update(status='invalidated_visual_floor_geometry',successor_calibration=str(output))
    save(parent/'state.json',original)
    save(parent/'learning_log.json',[dict(number=j['number'],scenario=j['scenario'],
        outcome=j['outcome'],policy_training_sample_valid=False) for j in original['jobs'] if j['status']=='done'])
    state = dict(status='running_floor_calibration',generation=0,phase='diagnostic',
                 training_samples=False,generalization_verified=False,jobs=[],
                 candidate_source_hashes=source_hashes(workspace),parent_campaign=str(parent))
    for index,baseline in enumerate(original['jobs'][:2]):
        case = output/f'case_{index}'
        case.mkdir()
        shutil.copy2(Path(baseline['output'])/'policy.json',case/'policy.json')
        manifest = json.loads((Path(baseline['output'])/'manifest.json').read_text())
        manifest['policy_sha256'] = digest(case/'policy.json')
        save(case/'manifest.json',manifest)
        state['jobs'].append(dict(number=index,generation=0,phase='diagnostic',role='candidate',
            output=str(case),scenario=baseline['scenario'],parameters=baseline['parameters'],
            status='prepared',unit=f'osp-floor-pose-case-{index}.service',
            baseline_infrastructure_invalid=True,workspace=str(workspace)))
    path = output/'state.json'
    save(path,state)
    while any(j['status']!='done' for j in state['jobs']):
        if source_hashes(workspace)!=state['candidate_source_hashes']:
            raise RuntimeError('Frozen floor runtime changed')
        for job in state['jobs']:
            if job['status']=='running' and not service_active(job['unit']):
                collect(job)
                save(path,state)
        active = [j for j in state['jobs'] if j['status']=='running']
        slots = max(0,2-max(len(native_processes()),len(active)))
        domains = {j.get('domain') for j in active}
        for job in [j for j in state['jobs'] if j['status']=='prepared'][:slots]:
            job['domain'] = next(d for d in (116,117) if d not in domains)
            domains.add(job['domain'])
            subprocess.run(['/usr/bin/python3',str(ROOT/'training_robust/run_native_service.py'),
                '--unit',job['unit'].removesuffix('.service'),'--workspace',str(workspace),
                '--output',job['output'],'--','/usr/bin/python3',str(ROOT/'training_robust/gazebo_benchmark.py'),
                '--output',job['output'],'--run-prepared','--domain',str(job['domain']),'--record-video'],
                check=True,capture_output=True,text=True)
            job['status']='running'
            save(path,state)
        for job in [j for j in state['jobs'] if j['status']=='running' and 'floor_sensor_gate' not in j]:
            episode = Path(job['output'])/'episodes'/str(job['scenario']['seed'])
            live = episode/'live_status.json'
            if not live.exists() or json.loads(live.read_text()).get('status',{}).get('state')!='running':
                continue
            measured = probe(job['domain'],Path(job['output'])/'floor_probe.json')
            job['floor_sensor_gate'] = floor_gate(measured)
            save(path,state)
            if not job['floor_sensor_gate']:
                raise RuntimeError(f'Corrected floor sensor gate failed for {job["number"]}')
        time.sleep(5)
    state['floor_sensor_calibration_passed'] = all(j.get('floor_sensor_gate',False) for j in state['jobs'])
    state['status'] = 'completed_calibration_requires_fresh_training'
    save(path,state)
    save(output/'comparison.json',[dict(seed=j['scenario']['seed'],shape=j['scenario']['arena']['shape'],
        baseline_infrastructure_invalid=True,candidate=j['outcome'],floor_sensor_gate=j.get('floor_sensor_gate'))
        for j in state['jobs']])


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--parent',type=Path,required=True)
    parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    run(args.parent.resolve(),args.workspace.resolve(),args.output.resolve())
