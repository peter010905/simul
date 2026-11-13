"""Insert one controlled face-orientation comparison without killing native runs.

The older integrated dispatcher is suspended until this single comparison ends;
its frozen source, active missions and remaining prepared scenes are preserved.
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

import run_encoder_delta_regression as runner
from continual_native_curriculum import digest,save,source_hashes
from native_tournament import service_active
from run_directed_integrated_screen_v2 import prime_parent_inputs
from run_windowed_witness_regression_v2 import settle_claims
from launch_spatial_cem_v6 import start_unit

ROOT=Path(__file__).resolve().parents[1]
UNIT='osp-face-intent-v2.service'


def restore_screen(unit,screen):
    if service_active(unit):return
    result=subprocess.run(['systemctl','--user','start',unit],capture_output=True,text=True)
    if result.returncode or not service_active(unit):
        start_unit(unit,['/usr/bin/python3','-u',str(ROOT/'training_robust/run_directed_integrated_screen_v2.py'),str(screen)],screen/'coordinator_v2.log')
    if not service_active(unit):raise RuntimeError('Original integrated supervisor did not resume')


def prepare(parent,screen,dependency,workspace,output):
    old=json.loads((screen/'state.json').read_text());reference=old['jobs'][0]
    if reference['status']!='done' or reference['outcome']['success']:
        raise ValueError('Require the completed integrated rectangle failure')
    if '348 passed' not in (workspace.parent/'build_and_tests.log').read_text():
        raise ValueError('Require complete runtime checks')
    before=source_hashes(Path(reference['workspace']));after=source_hashes(workspace)
    changes=sorted(p for p in before.keys()|after.keys() if before.get(p)!=after.get(p))
    if changes!=['osp_mission/face_directed_look.py','osp_mission/mission.py','test/test_face_directed_runtime.py']:
        raise ValueError(f'Unexpected controlled changes:{changes}')
    output.mkdir(parents=True,exist_ok=False);case=output/'case_0';case.mkdir()
    shutil.copy2(Path(reference['output'])/'policy.json',case/'policy.json')
    manifest=json.loads((Path(reference['output'])/'manifest.json').read_text())
    assert manifest['policy_sha256']==digest(case/'policy.json')
    save(case/'manifest.json',manifest)
    files=[Path(__file__),ROOT/'training_robust/run_encoder_delta_regression.py',
        ROOT/'training_robust/run_directed_integrated_screen_v2.py',
        ROOT/'scripts/prepare_face_directed_runtime.py',ROOT/'scripts/candidate_face_directed_look.py']
    save(output/'state.json',dict(status='running',phase='diagnostic',coordinator_unit=UNIT,
        source_hashes=after,driver_hashes={str(p.resolve()):digest(p) for p in files},
        parent_campaign=str(parent),suspended_screen=str(screen),dependency=str(dependency),
        baseline=reference['outcome'],source_changes=changes,training_samples=False,
        generalization_verified=False,production_promoted=False,
        comparison_limit='Same world,weights and integrated source;only face-observation orientation handling differs. One repeat is not a generalization or isolated timing estimate.',
        jobs=[dict(number=0,generation=0,phase='diagnostic',role='candidate',workspace=str(workspace),
            output=str(case),scenario=reference['scenario'],parameters=reference['parameters'],
            status='prepared',domain=136,unit='osp-face-intent-v2-native.service')]))


def run(parent,screen,dependency,workspace,output):
    state=json.loads((output/'state.json').read_text())
    if state['jobs'][0]['status']=='done':return
    runner.wait_for_dependency(dependency,output)
    original=json.loads((screen/'state.json').read_text());owner=json.loads((parent/'state.json').read_text())
    old_unit=original['coordinator_unit']
    if not service_active(old_unit):raise RuntimeError('Require verified current integrated supervisor before handoff')
    save(output/'screen_before_reservation.json',original)
    try:
        subprocess.run(['systemctl','--user','stop',old_unit],check=True)
        if service_active(owner['coordinator_unit']):
            subprocess.run(['systemctl','--user','stop',owner['coordinator_unit']],check=True)
        settle_claims(parent,screen,output)
        prime_parent_inputs(parent)
        # A currently running integrated episode already passed all input
        # probes; do not wait for a slot with unmeasured effective inputs.
        for job in original['jobs']:
            if job['status']=='running':
                pid=runner.native_pid(job['unit'],Path(job['output']))
                if pid and not all(job.get(k) for k in ('floor_sensor_gate','runtime_configuration_gate','velocity_profile_gate','slam_transport_gate')):
                    raise RuntimeError('Existing integrated native needs its input probes collected before reservation')
        save(output/'slot_ownership.json',dict(suspended_dispatcher=old_unit,
            existing_native_units_preserved=True,parent_inputs_primed=True))
        # Reuse the frozen single-trial primitive with the identical effective
        # encoder/SLAM input gates. Its preparation function is not called.
        runner.run(parent,dependency,workspace,output)
        state=json.loads((output/'state.json').read_text());job=state['jobs'][0]
        report=json.loads((Path(job['output'])/'report.json').read_text())['rows'][0]['result']
        profile=report.get('face_directed_look',{})
        job['face_directed_profile_gate']=profile.get('version')==1 and profile.get('creates_coverage_or_recognition_credit') is False
        job['face_directed_execution']=profile
        if not job['face_directed_profile_gate']:
            job['raw_outcome_before_face_profile']=dict(job['outcome'])
            job['outcome'].update(success=False,infrastructure_failure=True,reason='face_directed_profile_unverified')
        state['status']='completed_face_intent_regression';save(output/'state.json',state)
    finally:
        # Resume the old dispatcher first; it owns collection of its original
        # active episode and reserves its next slot before resuming CEM.
        settle_claims(parent,screen,output)
        restore_screen(old_unit,screen)
        save(output/'restored_dispatcher.json',dict(unit=old_unit,active=True,original_prepared_scenes_preserved=True))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('parent','screen','dependency','workspace','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--prepare-only',action='store_true');args=parser.parse_args()
    paths={name:getattr(args,name).resolve() for name in ('parent','screen','dependency','workspace','output')}
    if args.prepare_only:prepare(**paths)
    else:run(**paths)
