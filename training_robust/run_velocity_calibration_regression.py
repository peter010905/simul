"""One-slot native fixed-velocity-calibration ablation;retain CEM state and existing runs."""
import argparse
import json
from pathlib import Path
import shutil
import threading

from continual_native_curriculum import digest,save,source_hashes
from observe_spatial_campaign import native_pid
from probe_velocity_profile import EXPECTED,gate,probe
from run_one_slot_reacquisition import run as run_reserved


def prepare(parent,prior,workspace,model,output):
    baseline=json.loads((prior/'state.json').read_text())['jobs'][0]
    if baseline['status']!='done' or baseline['outcome']['success'] or baseline['outcome']['approached']!=3:
        raise ValueError('Require preserved three-ID/three-approach localization failure')
    before=source_hashes(Path(baseline['workspace']));after=source_hashes(workspace)
    changed={key for key in before.keys()|after.keys() if before.get(key)!=after.get(key)}
    if changed!={'setup.py','launch/control.launch.py','config/sim_ekf.yaml',
            'osp_mission/calibrated_wheel_twist.py','test/test_velocity_calibration_runtime.py','test/test_sim_state_estimation.py'}:
        raise ValueError(f'Unexpected estimator calibration changes:{changed}')
    fitted=json.loads(model.read_text())
    if fitted['rounded_candidate_scale']!=EXPECTED['calibrated_wheel_twist']['linear_scale']:
        raise ValueError('Preserve fitted calibration provenance')
    if '235 passed' not in (workspace.parent/'build_and_tests.log').read_text():
        raise ValueError('Require staged build and complete runtime tests')
    output.mkdir(parents=True,exist_ok=False);case=output/'case_0';case.mkdir()
    shutil.copy2(Path(baseline['output'])/'policy.json',case/'policy.json')
    manifest=json.loads((Path(baseline['output'])/'manifest.json').read_text())
    manifest['policy_sha256']=digest(case/'policy.json');save(case/'manifest.json',manifest)
    save(output/'state.json',dict(status='running',phase='diagnostic',training_samples=False,
        generalization_verified=False,production_promoted=False,parent_campaign=str(parent),
        coordinator_unit='osp-velocity-calibration-v1.service',prior_diagnostic=str(prior),
        baseline=baseline['outcome'],source_hashes=after,expected_velocity_profile=EXPECTED,
        calibration_model=str(model),calibration_model_sha256=digest(model),
        jobs=[dict(number=0,generation=0,phase='diagnostic',role='candidate',workspace=str(workspace),
            output=str(case),scenario=baseline['scenario'],parameters=baseline['parameters'],
            status='prepared',domain=118,unit='osp-velocity-calibration-v1-native.service')]))


def run(parent,prior,workspace,model,output):
    if not (output/'state.json').exists():prepare(parent,prior,workspace,model,output)
    stop=threading.Event()
    def measure():
        while not stop.wait(3):
            state=json.loads((output/'state.json').read_text());job=state['jobs'][0]
            if job['status']=='done':return
            try:
                if job['status']!='running' or native_pid(job['unit'],Path(job['output'])) is None:continue
                live=Path(job['output'])/'episodes'/str(job['scenario']['seed'])/'live_status.json'
                if not live.exists() or json.loads(live.read_text())['status']['state']!='running':continue
                if gate(probe(job['domain'],output/'effective_velocity_profile.json')):return
            except Exception as exc:
                save(output/'velocity_observation_error.json',dict(error=repr(exc),native_not_restarted=True))
    thread=threading.Thread(target=measure,daemon=True);thread.start()
    try:run_reserved(parent,workspace,output)
    finally:stop.set();thread.join(timeout=35)
    state=json.loads((output/'state.json').read_text());job=state['jobs'][0];observed=output/'effective_velocity_profile.json'
    state['velocity_profile_verified']=gate(json.loads(observed.read_text())) if observed.exists() else False
    if not state['velocity_profile_verified']:
        job['raw_outcome_before_velocity_gate']=dict(job['outcome'])
        job['outcome'].update(success=False,infrastructure_failure=True,reason='velocity_profile_unverified')
    state['status']='completed_velocity_calibration_regression';save(output/'state.json',state)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['parent','prior','workspace','model','output']:parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();run(args.parent.resolve(),args.prior.resolve(),args.workspace.resolve(),args.model.resolve(),args.output.resolve())
