"""One-slot native trial of state-driven startup plus landmark seed recovery."""
import argparse
import json
from pathlib import Path
import shutil

from continual_native_curriculum import digest,save,source_hashes
from run_one_slot_reacquisition import run as run_reserved


def prepare(parent,workspace,output):
    original=json.loads((parent/'state.json').read_text())['jobs'][0]
    if original['status']!='done' or original['outcome']['reason']!='timeout':
        raise ValueError('Preserve the original failed circle as the diagnostic reference')
    before=source_hashes(Path(original['workspace']));after=source_hashes(workspace)
    changes={k for k in before.keys()|after.keys() if before.get(k)!=after.get(k)}
    expected={'setup.py','launch/robot.launch.py','launch/simulation.launch.py',
        'osp_mission/detector.py','osp_mission/landmark_reacquisition.py',
        'osp_mission/startup_guard.py','osp_mission/startup_state.py',
        'test/test_reacquisition_runtime.py','test/test_startup_state_runtime.py','test/test_startup_guard_ros_runtime.py'}
    if changes!=expected:raise ValueError(f'Unexpected startup/seed candidate changes:{changes}')
    if '231 passed' not in (workspace.parent/'build_and_tests.log').read_text():
        raise ValueError('Require all runtime and real-ROS startup smoke checks')
    output.mkdir(parents=True,exist_ok=False);case=output/'case_0';case.mkdir()
    shutil.copy2(Path(original['output'])/'policy.json',case/'policy.json')
    manifest=json.loads((Path(original['output'])/'manifest.json').read_text())
    manifest['policy_sha256']=digest(case/'policy.json');save(case/'manifest.json',manifest)
    state=dict(status='running',phase='diagnostic',training_samples=False,generalization_verified=False,
        purpose='Same failed circle/weights;state-driven SLAM startup then standard Nav2;three-frame unconfirmed seed recovery',
        coordinator_unit='osp-seed-startup-v2.service',parent_campaign=str(parent),baseline=original['outcome'],
        source_hashes=after,jobs=[dict(number=0,generation=0,phase='diagnostic',role='candidate',
            workspace=str(workspace),output=str(case),scenario=original['scenario'],parameters=original['parameters'],
            status='prepared',domain=116,unit='osp-seed-startup-v2-native.service')])
    save(output/'state.json',state)


def run(parent,workspace,output):
    if not (output/'state.json').exists():prepare(parent,workspace,output)
    run_reserved(parent,workspace,output)
    state=json.loads((output/'state.json').read_text());job=state['jobs'][0]
    report=Path(job['output'])/'episodes'/str(job['scenario']['seed'])/'startup_guard.json'
    state['observed_startup_guard']=json.loads(report.read_text()) if report.exists() else None
    state['startup_guard_verified']=bool(state['observed_startup_guard'] and state['observed_startup_guard'].get('success'))
    if not state['startup_guard_verified']:
        job['raw_outcome_before_startup_guard_check']=dict(job['outcome'])
        job['outcome'].update(success=False,infrastructure_failure=True,reason='startup_guard_unverified')
    state['status']='completed_startup_and_seed_regression';save(output/'state.json',state)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('parent','workspace','output'):parser.add_argument('--'+key,type=Path,required=True)
    args=parser.parse_args();run(args.parent.resolve(),args.workspace.resolve(),args.output.resolve())
