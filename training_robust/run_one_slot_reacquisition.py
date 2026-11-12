"""Reserve one native slot for a seed-recovery diagnostic, then resume CEM.

Existing native services, frozen sources, prepared maps and CEM state remain
unchanged. Only the dispatcher is stopped until the diagnostic claims a slot.
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import threading
import time

from continual_native_curriculum import digest,save,source_hashes
from native_tournament import service_active
from observe_spatial_campaign import native_pid
from run_fresh_variant_probe import run as run_probe

ROOT=Path(__file__).resolve().parents[1]


def prepare(parent,workspace,output):
    previous=json.loads((parent/'state.json').read_text());failed=previous['jobs'][0]
    if failed['status']!='done' or failed['outcome']['reason']!='timeout':
        raise ValueError('Require preserved finalized native failure')
    reference=Path(failed['workspace']);before=source_hashes(reference);after=source_hashes(workspace)
    changes={k for k in before.keys()|after.keys() if before.get(k)!=after.get(k)}
    if changes!={'osp_mission/detector.py','osp_mission/landmark_reacquisition.py','test/test_reacquisition_runtime.py'}:
        raise ValueError(f'Unexpected candidate changes:{changes}')
    if '223 passed' not in (workspace.parent/'build_and_tests.log').read_text():
        raise ValueError('Require completed candidate build/runtime checks')
    output.mkdir(parents=True,exist_ok=False);case=output/'case_0';case.mkdir()
    shutil.copy2(Path(failed['output'])/'policy.json',case/'policy.json')
    manifest=json.loads((Path(failed['output'])/'manifest.json').read_text())
    manifest['policy_sha256']=digest(case/'policy.json');save(case/'manifest.json',manifest)
    state=dict(status='running',phase='diagnostic',training_samples=False,generalization_verified=False,
        purpose='Same failed circle/weights;recover sub-threshold landmark seeds with3fresh readable frames',
        coordinator_unit='osp-seed-reacquisition-regression-v1.service',parent_campaign=str(parent),
        baseline=failed['outcome'],source_hashes=after,jobs=[dict(number=0,generation=0,phase='diagnostic',
            role='candidate',workspace=str(workspace),output=str(case),scenario=failed['scenario'],
            parameters=failed['parameters'],status='prepared',domain=116,
            unit='osp-seed-reacquisition-regression-v1-native.service')])
    save(output/'state.json',state)


def resume_parent(parent):
    state=json.loads((parent/'state.json').read_text())
    if state['status']!='running':raise RuntimeError('Parent requires attention; do not override its state')
    unit=state['coordinator_unit']
    if service_active(unit):return
    load=subprocess.run(['systemctl','--user','show',unit,'-p','LoadState','--value'],
        capture_output=True,text=True,check=True).stdout.strip()
    if load=='loaded':
        started=subprocess.run(['systemctl','--user','start',unit],capture_output=True,text=True)
        if started.returncode==0 and service_active(unit):return
    subprocess.run(['systemd-run','--user','--unit='+unit.removesuffix('.service'),
        f'--property=WorkingDirectory={ROOT}','--property=MemoryMax=512M','--property=Restart=no',
        f'--property=StandardOutput=append:{parent/"coordinator.log"}',
        f'--property=StandardError=append:{parent/"coordinator.log"}',
        '/usr/bin/python3','-u',str(ROOT/'training_robust/spatial_native_curriculum_v5.py'),str(parent)],
        check=True,capture_output=True,text=True)
    if not service_active(unit):raise RuntimeError('Parent dispatcher did not become active')


def run(parent,workspace,output):
    if not (output/'state.json').exists():prepare(parent,workspace,output)
    state=json.loads((output/'state.json').read_text())
    if state['jobs'][0]['status']=='done':return
    parent_state=json.loads((parent/'state.json').read_text())
    if parent_state['status']!='running':raise RuntimeError('Require active parent CEM before reserving a slot')
    subprocess.run(['systemctl','--user','stop',parent_state['coordinator_unit']],check=True)
    stop=threading.Event()
    def watcher():
        while not stop.wait(2):
            try:
                current=json.loads((output/'state.json').read_text());job=current['jobs'][0]
                if (job['status']=='running' and native_pid(job['unit'],Path(job['output'])) is not None) or job['status']=='done':
                    resume_parent(parent)
                    save(output/'dispatch_handoff.json',dict(status='original_CEM_resumed',
                        original_native_runs_preserved=True,new_slot_claimed_by=job['unit']))
                    return
            except Exception as exc:
                save(output/'dispatch_handoff.json',dict(status='resume_retry',error=repr(exc)))
    thread=threading.Thread(target=watcher,daemon=True);thread.start()
    try:
        run_probe(argparse.Namespace(output=output,workspace=workspace))
        state=json.loads((output/'state.json').read_text());state['status']='completed_seed_reacquisition_regression'
        save(output/'state.json',state)
    finally:
        stop.set();thread.join(timeout=10);resume_parent(parent)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('parent','workspace','output'):parser.add_argument('--'+key,type=Path,required=True)
    args=parser.parse_args();run(args.parent.resolve(),args.workspace.resolve(),args.output.resolve())
