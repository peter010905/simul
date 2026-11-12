"""One-slot same-scene SLAM heading-prior comparison; retain all CEM learning."""
import argparse
import json
from pathlib import Path
import shutil
import threading

import yaml

from continual_native_curriculum import digest,save,source_hashes
from observe_spatial_campaign import native_pid
from probe_heading_prior import EXPECTED,gate,probe
from run_one_slot_reacquisition import run as run_reserved


def prepare(parent,prior,workspace,output):
    baseline=json.loads((prior/'state.json').read_text())['jobs'][0]
    if baseline['status']!='done' or baseline['outcome']['success'] or baseline['outcome']['approached']!=3:
        raise ValueError('Require finalized three-ID/three-approach coordinate failure as reference')
    before=source_hashes(Path(baseline['workspace']));after=source_hashes(workspace)
    if {k for k in before.keys()|after.keys() if before.get(k)!=after.get(k)}!={'config/slam.yaml'}:
        raise ValueError('Only the SLAM heading-prior profile may change')
    old=yaml.safe_load((Path(baseline['workspace'])/'src/osp_mission/config/slam.yaml').read_text())
    new=yaml.safe_load((workspace/'src/osp_mission/config/slam.yaml').read_text())
    for key,value in EXPECTED.items():
        if new['slam_toolbox']['ros__parameters'][key]!=value:raise ValueError('Wrong heading prior')
        old['slam_toolbox']['ros__parameters'].pop(key);new['slam_toolbox']['ros__parameters'].pop(key)
    if old!=new:raise ValueError('Unrelated SLAM setting changed')
    if '231 passed' not in (workspace.parent/'build_and_tests.log').read_text():
        raise ValueError('Require complete staged build/runtime checks')
    output.mkdir(parents=True,exist_ok=False);case=output/'case_0';case.mkdir()
    shutil.copy2(Path(baseline['output'])/'policy.json',case/'policy.json')
    manifest=json.loads((Path(baseline['output'])/'manifest.json').read_text())
    manifest['policy_sha256']=digest(case/'policy.json');save(case/'manifest.json',manifest)
    save(output/'state.json',dict(status='running',phase='diagnostic',training_samples=False,
        generalization_verified=False,production_promoted=False,parent_campaign=str(parent),
        coordinator_unit='osp-heading-prior-v1.service',prior_diagnostic=str(prior),
        baseline=baseline['outcome'],source_hashes=after,expected_heading_prior=EXPECTED,
        jobs=[dict(number=0,generation=0,phase='diagnostic',role='candidate',workspace=str(workspace),
            output=str(case),scenario=baseline['scenario'],parameters=baseline['parameters'],
            status='prepared',domain=117,unit='osp-heading-prior-v1-native.service')]))


def run(parent,prior,workspace,output):
    if not (output/'state.json').exists():prepare(parent,prior,workspace,output)
    stop=threading.Event()
    def measure():
        while not stop.wait(3):
            state=json.loads((output/'state.json').read_text());job=state['jobs'][0]
            if job['status']=='done':return
            try:
                if job['status']!='running' or native_pid(job['unit'],Path(job['output'])) is None:continue
                live=Path(job['output'])/'episodes'/str(job['scenario']['seed'])/'live_status.json'
                if not live.exists() or json.loads(live.read_text())['status']['state']!='running':continue
                report=probe(job['domain'],output/'effective_heading_prior.json')
                if gate(report):return
            except Exception as exc:
                save(output/'heading_observation_error.json',dict(error=repr(exc),native_not_restarted=True))
    thread=threading.Thread(target=measure,daemon=True);thread.start()
    try:run_reserved(parent,workspace,output)
    finally:stop.set();thread.join(timeout=35)
    state=json.loads((output/'state.json').read_text());job=state['jobs'][0]
    observed=output/'effective_heading_prior.json'
    state['heading_prior_verified']=gate(json.loads(observed.read_text())) if observed.exists() else False
    if not state['heading_prior_verified']:
        job['raw_outcome_before_heading_gate']=dict(job['outcome'])
        job['outcome'].update(success=False,infrastructure_failure=True,reason='heading_prior_unverified')
    state['status']='completed_heading_prior_regression';save(output/'state.json',state)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['parent','prior','workspace','output']:parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();run(args.parent.resolve(),args.prior.resolve(),args.workspace.resolve(),args.output.resolve())
