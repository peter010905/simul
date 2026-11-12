"""Finite native ablation on a preserved failed scene; no training samples.

Drain the preceding pair, retain videos and independent audits, then compare
live local costmaps with and without frame-invariant physical approach proof.
The caller decides whether a candidate merits a fresh-map learning campaign.
"""
import argparse
import copy
import datetime
import fcntl
import json
from pathlib import Path
import subprocess
import time

from continual_native_curriculum import digest, save, source_hashes
from native_tournament import native_processes, service_active
from run_floor_pose_regression import floor_gate, probe
from runtime_configuration_v2 import probe as configuration_probe, configuration_gate
from spatial_native_curriculum_v3 import collect

ROOT=Path(__file__).resolve().parents[1]


def prepare(parent,output,workspaces):
    output.mkdir(parents=True,exist_ok=False)
    previous=json.loads((parent/'state.json').read_text())
    if previous['status']!='held_for_navigation_regression':
        raise RuntimeError('Stop only the previous dispatcher; preserve its native jobs')
    original=previous['jobs'][0]
    if original['outcome']['success'] or original['scenario']['seed']!=1159000000:
        raise RuntimeError('This ablation requires the independently preserved circle failure')
    state=dict(status='waiting_for_slots',phase='diagnostic',training_samples=False,
        generalization_verified=False,parent_campaign=str(parent),jobs=[],
        coordinator_unit='osp-navigation-regression-v1.service',source_hashes={
            str(w):source_hashes(w) for w in workspaces},driver_hashes={
                str(p):digest(p) for p in [Path(__file__).resolve(),
                    ROOT/'training_robust/runtime_configuration_v2.py']},
        baseline=copy.deepcopy(original['outcome']))
    for i,workspace in enumerate(workspaces):
        case=output/f'case_{i}';case.mkdir()
        (case/'policy.json').write_bytes((Path(original['output'])/'policy.json').read_bytes())
        manifest=json.loads((Path(original['output'])/'manifest.json').read_text())
        manifest['policy_sha256']=digest(case/'policy.json');save(case/'manifest.json',manifest)
        state['jobs'].append(dict(number=i,phase='diagnostic',role='candidate',
            workspace=str(workspace),output=str(case),scenario=original['scenario'],
            parameters=original['parameters'],status='prepared',
            unit=f'osp-navigation-regression-v1-{i}.service'))
    save(output/'state.json',state)


def collect_ready(state):
    for job in state['jobs']:
        if job['status']=='prepared' and (Path(job['output'])/'service.json').exists():
            job['status']='running'
        if job['status']=='running' and not service_active(job['unit']):
            collect(job)
            if not (job.get('floor_sensor_gate') and job.get('runtime_configuration_gate')):
                job['raw_outcome_before_input_gates']=dict(job['outcome'])
                job['outcome'].update(success=False,infrastructure_failure=True,
                    reason='native_input_or_local_costmap_configuration_unverified')


def run(parent,output):
    lock=(output/'coordinator.lock').open('w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    state=json.loads((output/'state.json').read_text())
    while any(j['status']!='done' for j in state['jobs']):
        for workspace,expected in state['source_hashes'].items():
            if source_hashes(Path(workspace))!=expected:raise RuntimeError('Frozen candidate changed')
        for path,expected in state['driver_hashes'].items():
            if digest(Path(path))!=expected:raise RuntimeError('Frozen regression driver changed')
        # The stopped predecessor dispatcher cannot race these state updates.
        previous=json.loads((parent/'state.json').read_text())
        if previous['status']!='held_for_navigation_regression':
            raise RuntimeError('Predecessor campaign state changed while diagnostic owns dispatch')
        collect_ready(previous);save(parent/'state.json',previous)
        collect_ready(state);save(output/'state.json',state)
        active=[j for j in state['jobs'] if j['status']=='running']
        for job in active:
            directory=Path(job['output']);live=directory/'episodes'/str(job['scenario']['seed'])/'live_status.json'
            if not live.exists() or json.loads(live.read_text()).get('status',{}).get('state')!='running':continue
            if 'floor_sensor_gate' not in job:
                measured=probe(job['domain'],directory/'floor_probe.json')
                job['floor_sensor_gate']=floor_gate(measured)
                save(output/'state.json',state)
                if not job['floor_sensor_gate']:raise RuntimeError('Native cliff inputs invalid')
            if not job.get('runtime_configuration_gate'):
                measured=configuration_probe(job['domain'],directory/'runtime_configuration.json')
                check=configuration_gate(measured,directory/'policy.json',1000)
                save(directory/'runtime_configuration_check.json',check)
                job['runtime_configuration_gate']=check['passed']
                save(output/'state.json',state)
                if check['mismatches']:raise RuntimeError('Effective local costmap configuration mismatch')
        slots=max(0,2-max(len(native_processes()),len(active)))
        domains={j.get('domain') for j in active}
        for job in [j for j in state['jobs'] if j['status']=='prepared'][:slots]:
            domain=next(d for d in (116,117) if d not in domains);domains.add(domain)
            result=subprocess.run(['/usr/bin/python3',str(ROOT/'training_robust/run_native_service.py'),
                '--unit',job['unit'].removesuffix('.service'),'--workspace',job['workspace'],
                '--output',job['output'],'--','/usr/bin/python3',str(ROOT/'training_robust/gazebo_benchmark.py'),
                '--output',job['output'],'--run-prepared','--domain',str(domain),'--record-video'],
                capture_output=True,text=True)
            (Path(job['output'])/'service_launch.log').write_text(result.stdout+result.stderr)
            if result.returncode:raise RuntimeError('Native regression launch failed')
            job.update(status='running',domain=domain)
            state['status']='running_navigation_regression';save(output/'state.json',state)
        time.sleep(5)
    state.update(status='completed_navigation_regression',completed_utc=datetime.datetime.now(
        datetime.timezone.utc).isoformat(),generalization_verified=False)
    save(output/'state.json',state)
    save(output/'comparison.json',dict(baseline=state['baseline'],training_samples=False,
        results=[dict(workspace=j['workspace'],outcome=j['outcome'],floor_sensor_gate=j.get('floor_sensor_gate'),
            runtime_configuration_gate=j.get('runtime_configuration_gate')) for j in state['jobs']]))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--parent',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--workspaces',type=Path,nargs=2,required=True)
    args=parser.parse_args();parent=args.parent.resolve();output=args.output.resolve()
    if not (output/'state.json').exists():prepare(parent,output,[p.resolve() for p in args.workspaces])
    run(parent,output)
