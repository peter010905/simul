"""Persistent training supervisor. Stops only at explicit measurable gates.

Passing the gates establishes empirical performance, never global optimality.
Fresh test scenes are never passed to CEM. Validation is used for checkpoint
selection; each round gets new training and test seeds and preserves all logs.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import yaml
from osp_mission.scenarios import make_scenario


def atomic_json(path,value):
    tmp=path.with_suffix('.tmp'); tmp.write_text(json.dumps(value,indent=2)); tmp.replace(path)


def main():
    p=argparse.ArgumentParser(); p.add_argument('--root',default='training_hidden'); p.add_argument('--workers',type=int,default=4)
    p.add_argument('--target',type=float,default=.90); p.add_argument('--initial',default='training_hidden/initial')
    p.add_argument('--skip-initial-wait',action='store_true'); p.add_argument('--max-rounds',type=int,default=0)
    a=p.parse_args(); root=Path(a.root).resolve(); root.mkdir(parents=True,exist_ok=True)
    lock=(root/'continual.lock').open('w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB); lock.write(str(os.getpid())); lock.flush()
    state=root/'live_state.json'; champion=root/'deployed_policy.json'; previous=Path(a.initial).resolve()
    stop_file=root/'STOP_AFTER_ROUND'; best_key=(-1,-1,-1000); rounds=0
    if not a.skip_initial_wait:
        copied_mtime=0.
        while not (previous/'test_report.json').exists():
            candidate=previous/'best_policy.json'
            if candidate.exists() and candidate.stat().st_mtime!=copied_mtime:
                try:
                    atomic_json(champion,json.loads(candidate.read_text())); copied_mtime=candidate.stat().st_mtime
                except (OSError,ValueError): pass
            atomic_json(state,dict(phase='initial_training',run=str(previous),round=0,target=a.target))
            time.sleep(3)
    if (previous/'best_policy.json').exists(): atomic_json(champion,json.loads((previous/'best_policy.json').read_text()))
    while not stop_file.exists() and (not a.max_rounds or rounds<a.max_rounds):
        rounds+=1; run=root/f'round_{rounds:03d}'; run.mkdir(exist_ok=True)
        scenes=run/'scenarios'; manifest={}
        for split,start,count in [('train',rounds*100000,120),('validation',10000,32),('test',5000000+rounds*100000,48)]:
            dest=scenes/split; dest.mkdir(parents=True,exist_ok=True); manifest[split]=list(range(start,start+count))
            for seed in manifest[split]:
                path=dest/f'{seed:07d}.yaml'
                if not path.exists(): path.write_text(yaml.safe_dump(make_scenario(seed),sort_keys=False))
        (scenes/'manifest.yaml').write_text(yaml.safe_dump(manifest))
        command=[sys.executable,'-m','osp_mission.train','--scenarios',str(scenes),'--output',str(run),
                 '--workers',str(a.workers),'--generations','24','--population','20','--batch','16','--seed',str(20261104+rounds)]
        if champion.exists(): command+=['--resume',str(champion)]
        atomic_json(state,dict(phase='training',run=str(run),round=rounds,target=a.target))
        with (run/'train.log').open('w') as f:
            process=subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT)
            while process.poll() is None:
                candidate=run/'best_policy.json'
                if candidate.exists():
                    try:
                        cfg=json.loads(candidate.read_text()); v=cfg['validation']
                        key=(v['success_rate'],v['mean_found'],-v['mean_seconds'])
                        if key>best_key:
                            best_key=key; atomic_json(champion,cfg)
                    except (OSError,ValueError,KeyError): pass
                time.sleep(3)
        if process.returncode:
            atomic_json(state,dict(phase='training_error_retry',run=str(run),round=rounds,exit_code=process.returncode)); time.sleep(10); continue
        report=json.loads((run/'test_report.json').read_text())
        if report['learned']['success_rate']<a.target or report['learned']['collisions']:
            atomic_json(state,dict(phase='gate_not_met_next_round',run=str(run),round=rounds,target=a.target,test=report['learned']))
            previous=run; continue
        # Freeze this checkpoint before independent real-image Gazebo validation.
        frozen=run/'gazebo_candidate.json'; shutil.copy2(run/'best_policy.json',frozen)
        results=[]
        for index,scenario in enumerate(sorted((scenes/'test').glob('*.yaml'))[:8]):
            atomic_json(state,dict(phase='gazebo_validation',run=str(run),round=rounds,episode=index+1,total=8))
            command=[sys.executable,'-m','osp_mission.gazebo_eval','--scenario',str(scenario),'--policy',str(frozen),
                     '--output',str(run/'gazebo'/f'{index:02d}'),'--domain','84']
            with (run/f'gazebo_{index:02d}.log').open('w') as f: completed=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT)
            evaluation=run/'gazebo'/f'{index:02d}'/'evaluation.json'
            results.append(json.loads(evaluation.read_text()) if completed.returncode==0 and evaluation.exists() else {'success':False,'reason':'evaluator_failure'})
        passed=sum(r.get('success',False) for r in results)>=8 and all(r.get('collisions',1)==0 for r in results)
        atomic_json(run/'gazebo_report.json',dict(episodes=results,passed=passed,required_successes=8,total=8))
        if passed:
            shutil.copy2(frozen,root/'verified_policy.json')
            atomic_json(state,dict(phase='empirical_gates_passed',run=str(run),round=rounds,globally_optimal_proven=False)); return
    atomic_json(state,dict(phase='stopped_after_round',round=rounds,requested_stop=stop_file.exists(),globally_optimal_proven=False))

if __name__=='__main__': main()
