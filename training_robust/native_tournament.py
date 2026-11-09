"""Bounded native policy search, followed by fresh-layout validation and a gated holdout.

Runs actual Gazebo services, at most two across this host's benchmark processes.
No policy receives evaluator ground truth. Failed and interrupted trials remain saved.
"""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

from audit_native_campaign import audit_episode

HERE = Path(__file__).resolve().parent


def save(path, value):
    tmp = path.with_name(path.name+'.tmp')
    tmp.write_text(json.dumps(value, indent=2)+'\n')
    tmp.replace(path)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def native_processes():
    result = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            argv = (proc/'cmdline').read_bytes().split(b'\0')
            if len(argv)>1 and argv[1].endswith(b'/gazebo_benchmark.py'):
                result.append(int(proc.name))
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            pass
    return result


def service_active(unit):
    result = subprocess.run(['systemctl','--user','show',unit,'-p','ActiveState','--value'],
                            text=True,capture_output=True,check=True)
    return result.stdout.strip() in ('active','activating','deactivating')


def ranking(jobs, candidates, phase):
    rows=[]
    for name in candidates:
        selected=[j for j in jobs if j['phase']==phase and j['candidate']==name]
        outcomes=[j.get('outcome',{}) for j in selected]
        successes=sum(bool(v.get('success')) for v in outcomes)
        rows.append(dict(candidate=name,planned=len(selected),finished=sum(j['status']=='done' for j in selected),
                         successes=successes,success_rate=successes/len(selected) if selected else 0.,
                         recognized=sum(v.get('targets',0) or 0 for v in outcomes),
                         approached=sum(v.get('approached',0) for v in outcomes),
                         mean_seconds=sum(v.get('seconds',600.) or 600. for v in outcomes)/max(1,len(selected))))
    return sorted(rows,key=lambda v:(-v['success_rate'],-v['recognized'],-v['approached'],v['mean_seconds'],v['candidate']))


def add_jobs(root, state, config, phase, names, scenes):
    for scene in scenes:
        for name in names:
            candidate=config['candidates'][name]
            number=len(state['jobs'])
            directory=root/'jobs'/f'{number:03d}_{phase}_{name}_{scene["seed"]}'
            directory.mkdir(parents=True,exist_ok=False)
            shutil.copy2(candidate['policy'],directory/'policy.json')
            manifest=dict(environment='gazebo_harmonic',planner='edge',camera_width=960,duration=600,
                          render_backend='software',start_docked=False,approach_radius_m=1.5,
                          near_id_confirmations=3,protocol_version=2,record_video=True,
                          policy_sha256=digest(directory/'policy.json'),scenarios=[scene])
            save(directory/'manifest.json',manifest)
            state['jobs'].append(dict(number=number,phase=phase,candidate=name,scenario=scene,
                                      output=str(directory),workspace=candidate['workspace'],
                                      unit=f'osp-nativesearch-{config["run_id"]}-{number:03d}.service',status='prepared'))


def check_sources(config):
    for name, expected in config.get('support_hashes',{}).items():
        if digest(Path(name)) != expected:
            raise RuntimeError('Frozen experiment driver or auditor changed')
    for candidate in config['candidates'].values():
        src=Path(candidate['workspace'])/'src/osp_mission'
        actual={str(p.relative_to(src)):digest(p) for p in src.rglob('*')
                if p.is_file() and not {'__pycache__','.pytest_cache'}.intersection(p.parts)}
        if actual!=candidate['source_hashes'] or digest(Path(candidate['policy']))!=candidate['policy_sha256']:
            raise RuntimeError('Frozen candidate source or policy changed')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path);args=parser.parse_args()
    root=args.root.resolve();config=json.loads((root/'configuration.json').read_text())
    statefile=root/'state.json'
    state=json.loads(statefile.read_text()) if statefile.exists() else dict(phase='screen',jobs=[],status='running')
    if not state['jobs']:
        add_jobs(root,state,config,'screen',list(config['candidates']),config['screen_scenes'])
        save(statefile,state)
    try:
        while state['status']=='running':
            check_sources(config)
            for job in state['jobs']:
                if job['status']!='running' or service_active(job['unit']):
                    continue
                directory=Path(job['output']);report=directory/'report.json'
                outcome=dict(success=False,targets=0,approached=0,seconds=600.,infrastructure_failure=True)
                if report.exists():
                    data=json.loads(report.read_text())
                    if len(data.get('rows',[]))==1:
                        result=data['rows'][0]['result']
                        audited=audit_episode(directory/'episodes'/str(job['scenario']['seed']),job['scenario'])
                        save(directory/'episode_audit.json',audited)
                        outcome.update(success=bool(result.get('success') and audited['success']),
                                       targets=result.get('targets',0),seconds=result.get('elapsed_seconds',600.),
                                       approached=result.get('ground_truth_audit',{}).get('approached_within_1_5m',0),
                                       reason=result.get('reason'),
                                       infrastructure_failure=result.get('reason') not in ('all_targets_found','timeout'))
                job.update(status='done',outcome=outcome)
                save(statefile,state)
                print(json.dumps(dict(completed=job['number'],candidate=job['candidate'],phase=job['phase'],**outcome)),flush=True)
            phase_jobs=[j for j in state['jobs'] if j['phase']==state['phase']]
            if phase_jobs and all(j['status']=='done' for j in phase_jobs):
                names=sorted({j['candidate'] for j in phase_jobs})
                ranked=ranking(state['jobs'],names,state['phase']);save(root/f'{state["phase"]}_ranking.json',ranked)
                if state['phase']=='screen':
                    state['phase']='validation'
                    finalists=config.get('validation_finalists',2)
                    add_jobs(root,state,config,'validation',[v['candidate'] for v in ranked[:finalists]],config['validation_scenes'])
                elif state['phase']=='validation' and not config.get('holdout_enabled',True):
                    state.update(status='completed_development_search',goal_verified=False)
                    selected=config['candidates'][ranked[0]['candidate']]
                    save(root/'selected_candidate.json',dict(candidate=ranked[0],
                        workspace=selected['workspace'],policy=selected['policy'],
                        status='development_selection_requires_broader_validation',
                        reserved_holdout_used=False))
                elif state['phase']=='validation' and ranked[0]['success_rate']>=.9:
                    state['phase']='holdout';state['frozen_final_candidate']=ranked[0]['candidate']
                    reserved=json.loads(Path(config['reserved_manifest']).read_text())
                    add_jobs(root,state,config,'holdout',[ranked[0]['candidate']],reserved['episodes'])
                    save(root/'final_candidate_frozen.json',dict(candidate=ranked[0],frozen_utc=datetime.datetime.now(datetime.timezone.utc).isoformat()))
                elif state['phase']=='holdout':
                    subprocess.run([sys.executable,str(HERE/'audit_final_native.py'),
                                    *[j['output'] for j in phase_jobs],'--reserved-manifest',config['reserved_manifest'],
                                    '--output',str(root/'final_gate.json')],check=False)
                    state['status']='completed';state['goal_verified']=json.loads((root/'final_gate.json').read_text())['achieved']
                else:
                    state['status']='completed_needs_improvement';state['goal_verified']=False
                save(statefile,state)
                continue
            recent=[j for j in state['jobs'] if j['status']=='done'][-3:]
            if len(recent)==3 and all(j['outcome']['infrastructure_failure'] for j in recent):
                state.update(status='stopped_infrastructure_failure',goal_verified=False);save(statefile,state);break
            if shutil.disk_usage(root).free < 8*1024**3:
                state.update(status='stopped_low_disk',goal_verified=False);save(statefile,state);break
            # Include the already-running independent experiment in the host limit.
            owned=sum(j['status']=='running' for j in state['jobs'])
            external=sum(service_active(u) for u in config.get('external_units',[]))
            slots=max(0,2-max(len(native_processes()),owned+external))
            active_domains={j.get('domain') for j in state['jobs'] if j['status']=='running'}
            for job in [j for j in state['jobs'] if j['status']=='prepared'][:slots]:
                domain=next(d for d in (110,111) if d not in active_domains)
                active_domains.add(domain);job['domain']=domain
                cmd=[sys.executable,str(HERE/'run_native_service.py'),'--unit',job['unit'].removesuffix('.service'),
                     '--workspace',job['workspace'],'--output',job['output'],'--',sys.executable,
                     str(HERE/'gazebo_benchmark.py'),'--output',job['output'],'--run-prepared',
                     '--domain',str(domain),'--record-video']
                result=subprocess.run(cmd,capture_output=True,text=True)
                (Path(job['output'])/'service_launch.log').write_text(result.stdout+result.stderr)
                if result.returncode:
                    raise RuntimeError(f'Native service launch failed for job {job["number"]}')
                job['status']='running';save(statefile,state)
            time.sleep(5.)
    except Exception as exc:
        state.update(status='coordinator_error',error=repr(exc),goal_verified=False);save(statefile,state);raise
    print(json.dumps({k:v for k,v in state.items() if k!='jobs'}),flush=True)


if __name__=='__main__':
    main()
