"""Resume the preserved screen with input collection before slot reservation.

The original v1 driver, native sources, prepared maps and outputs stay intact.
Only the supervisor is handed off, with the previous state saved verbatim.
"""
import argparse
import datetime
import fcntl
import json
from pathlib import Path
import subprocess
import time

import run_directed_integrated_screen as base
from probe_velocity_profile import probe as parent_velocity_probe, gate as parent_velocity_gate

UNIT='osp-directed-integrated-v2.service'


def prime_parent_inputs(parent):
    """Only the stopped dispatcher's owner writes its verified input fields."""
    path=parent/'state.json';state=json.loads(path.read_text())
    if base.service_active(state['coordinator_unit']):
        raise RuntimeError('Parent dispatcher must be stopped before collecting its state')
    for job in state['jobs']:
        if job['status']!='running':continue
        output=Path(job['output'])
        # Do not spend a slot waiting while a short mission can finish without
        # its floor/parameter/velocity probes. Observation gaps never restart it.
        while True:
            try:
                pid=base.native_pid(job['unit'],output)
                if pid is None and not base.service_active(job['unit']) and not base.benchmark_output_live(output):break
            except subprocess.TimeoutExpired as exc:
                base.save(output/'reservation_observation_gap.json',dict(error=repr(exc),native_not_restarted=True))
                time.sleep(3);continue
            live=output/'episodes'/str(job['scenario']['seed'])/'live_status.json'
            running=live.exists() and json.loads(live.read_text()).get('status',{}).get('state')=='running'
            if running and pid is not None:
                if not job.get('floor_sensor_gate'):
                    report=base.observe(base.floor_probe,job,'floor_probe.json')
                    if report is not None and base.floor_gate(report):job['floor_sensor_gate']=True
                if job.get('floor_sensor_gate') and not job.get('runtime_configuration_gate'):
                    report=base.observe(base.config_probe,job,'runtime_configuration.json')
                    if report is not None:
                        check=base.configuration_gate(report,output/'policy.json',1000)
                        base.save(output/'runtime_configuration_check.json',check)
                        if check['mismatches']:raise RuntimeError('Parent native input mismatch before slot reservation')
                        job['runtime_configuration_gate']=check['passed']
                if job.get('runtime_configuration_gate') and not job.get('velocity_profile_gate'):
                    report=base.observe(parent_velocity_probe,job,'effective_velocity_profile.json')
                    if report is not None:
                        job['velocity_profile_gate']=parent_velocity_gate(report)
                        if report.get('complete') and not job['velocity_profile_gate']:
                            raise RuntimeError('Parent effective velocity mismatch before slot reservation')
                if base.service_active(state['coordinator_unit']):
                    raise RuntimeError('Unexpected concurrent parent dispatcher; do not overwrite its state')
                base.save(path,state)
                if all(job.get(k) is True for k in ('floor_sensor_gate','runtime_configuration_gate','velocity_profile_gate')):
                    break
            time.sleep(3)
    base.save(parent/'slot_reservation_input_audit.json',dict(
        captured_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        jobs=[dict(number=j['number'],status=j['status'],input_gates={k:j.get(k) for k in
            ('floor_sensor_gate','runtime_configuration_gate','velocity_profile_gate')}) for j in state['jobs'] if j['status']=='running'],
        original_native_runs_preserved=True,no_retroactive_measurement_claim=True))


def adopt(output):
    path=output/'state.json';state=json.loads(path.read_text())
    if state['coordinator_unit']!=base.UNIT or base.service_active(base.UNIT):
        raise RuntimeError('Require the stopped v1 supervisor and its original state')
    for job in state['jobs']:
        if job['status']=='running' and base.native_pid(job['unit'],Path(job['output'])) is None:
            raise RuntimeError('Handoff requires a verified currently running native handle')
    backup=output/'state_before_supervisor_v2.json'
    if backup.exists():raise FileExistsError('Preserve the previous handoff')
    backup.write_bytes(path.read_bytes())
    state['coordinator_unit']=UNIT
    state['supervision_handoff']=dict(previous_unit=base.UNIT,new_unit=UNIT,previous_state_sha256=base.digest(backup),
        previous_state=str(backup),reason='Collect existing native inputs before waiting for a free slot',
        native_sources_and_scenarios_unchanged=True)
    state['driver_hashes'][str(Path(__file__).resolve())]=base.digest(Path(__file__))
    base.save(path,state)


def run(output):
    path=output/'state.json';lock=(output/'coordinator.lock').open('w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    state=json.loads(path.read_text())
    if state['coordinator_unit']!=UNIT:raise RuntimeError('Explicit supervisor handoff is required')
    parent=Path(state['parent_campaign']);dependency=Path(state['dependency']);held=Path(state['held_owner'])
    base.wait_for_dependency(dependency,output)
    for job in state['jobs']:
        if job['status']=='done':continue
        base.validate_held_owner(held)
        directory=Path(job['output']);workspace=Path(job['workspace'])
        owner=json.loads((parent/'state.json').read_text())
        if owner['status']!='running':raise RuntimeError('Parent campaign requires collection or repair first')
        restored=False
        try:
            # A verified inherited native already occupies its slot; no new
            # reservation or interruption of the parent is needed on handoff.
            if job['status']=='running' and base.native_pid(job['unit'],directory) is not None:
                base.resume_cem(parent);restored=True
            else:
                if base.service_active(owner['coordinator_unit']):
                    subprocess.run(['systemctl','--user','stop',owner['coordinator_unit']],check=True)
                base.settle_claims(parent,dependency,output)
                prime_parent_inputs(parent)
            while job['status']!='done':
                if base.source_hashes(workspace)!=state['source_hashes']:raise RuntimeError('Frozen native source changed')
                if any(base.digest(Path(p))!=h for p,h in state['driver_hashes'].items()):raise RuntimeError('Frozen supervisor source changed')
                if job['status']=='prepared' and (directory/'service.json').exists():job['status']='running'
                try:
                    pid=base.native_pid(job['unit'],directory) if job['status']=='running' else None
                    if pid and not restored:
                        base.resume_cem(parent);restored=True
                        base.save(directory/'dispatch_handoff.json',dict(verified_native_pid=pid,
                            resumed_unit=owner['coordinator_unit'],original_native_runs_preserved=True,
                            parent_input_gates_primed=True))
                    if job['status']=='running' and not base.service_active(job['unit']) and not base.benchmark_output_live(directory):
                        time.sleep(2);base.collect_screen(job);base.save(path,state)
                        print(json.dumps(dict(number=job['number'],phase=job['phase'],**job['outcome'])),flush=True)
                        break
                    if job['status']=='prepared' and len(base.native_processes())<2:
                        result=subprocess.run(['/usr/bin/python3',str(base.ROOT/'training_robust/run_native_service.py'),
                            '--unit',job['unit'].removesuffix('.service'),'--workspace',str(workspace),'--output',str(directory),
                            '--','/usr/bin/python3',str(base.ROOT/'training_robust/gazebo_benchmark.py'),
                            '--output',str(directory),'--run-prepared','--domain',str(job['domain']),'--record-video'],
                            capture_output=True,text=True)
                        (directory/'service_launch.log').write_text(result.stdout+result.stderr)
                        if result.returncode:raise RuntimeError('Native launch failed; original metadata retained')
                        job['status']='running';base.save(path,state)
                    if job['status']=='running':base.measure(job,pid)
                    base.save(path,state)
                except subprocess.TimeoutExpired as exc:
                    base.save(directory/'observation_gap.json',dict(error=repr(exc),native_not_restarted=True))
                time.sleep(3)
        finally:
            base.settle_claims(parent,dependency,output);base.resume_cem(parent)
    state['status']='completed_integrated_screen'
    state['results']={phase:base.quality([j['outcome'] for j in state['jobs'] if j['phase']==phase])
                     for phase in ('diagnostic','train')}
    base.save(path,state)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path);parser.add_argument('--adopt',action='store_true');args=parser.parse_args()
    if args.adopt:adopt(args.output.resolve())
    else:run(args.output.resolve())
