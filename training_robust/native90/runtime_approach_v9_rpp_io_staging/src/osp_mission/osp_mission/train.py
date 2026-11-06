"""Reproducible cross-entropy policy search and locked held-out evaluation."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import csv
from datetime import datetime,timezone
import hashlib
import json
import os
import shutil
from pathlib import Path
import time
import numpy as np
import yaml
from osp_mission.fast_sim import episode
from osp_mission.policy import BASELINE,FEATURES


def task(args):
    cfg,weights=args
    return episode(cfg,weights)


def evaluate(pool,scenarios,weights):
    return list(pool.map(task,[(cfg,np.asarray(weights).tolist()) for cfg in scenarios],chunksize=1))


def summary(rows):
    successes=[r for r in rows if r['success']]
    return dict(episodes=len(rows),success_rate=float(np.mean([r['success'] for r in rows])),
        mean_found=float(np.mean([r['found'] for r in rows])),mean_seconds=float(np.mean([r['seconds'] for r in rows])),
        successful_mean_seconds=float(np.mean([r['seconds'] for r in successes])) if successes else None,
        collisions=sum(r['collisions'] for r in rows),mean_reward=float(np.mean([r['reward'] for r in rows])),
        by_obstacles={str(n):dict(success_rate=float(np.mean([r['success'] for r in rows if r['difficulty']==n])),
            episodes=sum(r['difficulty']==n for r in rows)) for n in sorted(set(r['difficulty'] for r in rows))})


def write_policy(path,weights,metadata):
    payload=dict(version=1,features=FEATURES,parameters=np.asarray(weights).tolist(),
                 algorithm='cross_entropy_policy_search',environment='2d_sensor_surrogate',**metadata)
    tmp=path.with_suffix('.tmp'); tmp.write_text(json.dumps(payload,indent=2)); tmp.replace(path)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--scenarios',default='scenarios_hidden'); p.add_argument('--output',default='training_hidden/manual')
    p.add_argument('--generations',type=int,default=24); p.add_argument('--population',type=int,default=20)
    p.add_argument('--batch',type=int,default=16); p.add_argument('--workers',type=int,default=6)
    p.add_argument('--seed',type=int,default=20261104); p.add_argument('--resume',default='')
    a=p.parse_args(); dest=Path(a.output); dest.mkdir(parents=True,exist_ok=True)
    cfgs={split:[yaml.safe_load(f.read_text()) for f in sorted((Path(a.scenarios)/split).glob('*.yaml'))]
          for split in ['train','validation','test']}
    if not all(cfgs.values()): raise ValueError('Missing train/validation/test scenarios')
    sets=[set(s['seed'] for s in cfgs[split]) for split in cfgs]
    assert not (sets[0]&sets[1] or sets[0]&sets[2] or sets[1]&sets[2]),'Seed leakage'
    rng=np.random.default_rng(a.seed); mean=BASELINE.copy(); std=np.full(len(mean),1.5)
    if a.resume: mean=np.array(json.loads(Path(a.resume).read_text())['parameters'])
    metadata=dict(seed=a.seed,created_utc=datetime.now(timezone.utc).isoformat(),configuration=vars(a),
                  scenario_sha256={split:hashlib.sha256(json.dumps(cfgs[split],sort_keys=True).encode()).hexdigest() for split in cfgs})
    snapshot=dest/'source_snapshot'; snapshot.mkdir(exist_ok=True)
    metadata['source_sha256']={}
    for name in ['policy.py','fast_sim.py','scenarios.py','train.py']:
        source=Path(__file__).parent/name; shutil.copy2(source,snapshot/name)
        metadata['source_sha256'][name]=hashlib.sha256(source.read_bytes()).hexdigest()
    (dest/'configuration.json').write_text(json.dumps(metadata,indent=2))
    start=time.monotonic(); champion=mean.copy(); history=[]; trained_episodes=0
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        baseline_rows=evaluate(pool,cfgs['validation'],BASELINE)
        best=float(np.mean([r['reward'] for r in evaluate(pool,cfgs['validation'],champion)]))
        best_stats=summary(evaluate(pool,cfgs['validation'],champion))
        best_key=(best_stats['success_rate'],best_stats['mean_found'],-best_stats['mean_seconds'])
        write_policy(dest/'best_policy.json',champion,dict(validation=best_stats,generation=-1,training_episodes=0,**metadata))
        print(json.dumps(dict(event='baseline_validation',**summary(baseline_rows))),flush=True)
        for generation in range(a.generations):
            indices=rng.choice(len(cfgs['train']),min(a.batch,len(cfgs['train'])),replace=False)
            batch=[cfgs['train'][i] for i in indices]
            population=rng.normal(mean,std,size=(a.population,len(mean)))
            population=np.clip(population,-8,8); population[0]=champion; population[1]=mean
            work=[(cfg,weights.tolist()) for weights in population for cfg in batch]
            rows=list(pool.map(task,work,chunksize=1)); trained_episodes+=len(rows)
            scores=np.array([r['reward'] for r in rows]).reshape(a.population,len(batch)).mean(axis=1)
            elites=population[np.argsort(scores)[-max(3,a.population//4):]]
            mean=.25*mean+.75*elites.mean(axis=0); std=np.maximum(.20,.35*std+.65*elites.std(axis=0))
            candidate=population[int(scores.argmax())]
            validation=evaluate(pool,cfgs['validation'],candidate)
            stats=summary(validation)
            key=(stats['success_rate'],stats['mean_found'],-stats['mean_seconds'])
            if key>best_key:
                best_key=key
                best=stats['mean_reward']; champion=candidate.copy()
                write_policy(dest/'best_policy.json',champion,dict(validation=stats,generation=generation,training_episodes=trained_episodes,**metadata))
            entry=dict(generation=generation,train_best=float(scores.max()),validation=stats,best_validation_reward=best,
                       training_episodes=trained_episodes,wall_seconds=time.monotonic()-start)
            history.append(entry); (dest/'history.json').write_text(json.dumps(history,indent=2)); print(json.dumps(entry),flush=True)
            np.savez(dest/'checkpoint.npz',mean=mean,std=std,champion=champion,generation=generation,rng_state=json.dumps(rng.bit_generator.state))
        if not (dest/'best_policy.json').exists():
            write_policy(dest/'best_policy.json',champion,dict(training_episodes=trained_episodes,validation=summary(evaluate(pool,cfgs['validation'],champion)),**metadata))
        test_rows=evaluate(pool,cfgs['test'],champion); test_baseline=evaluate(pool,cfgs['test'],BASELINE)
        report=dict(environment='2d_sensor_surrogate',not_gazebo=True,training_episodes=trained_episodes,
            wall_seconds=time.monotonic()-start,baseline=summary(test_baseline),learned=summary(test_rows),
            acceptance=dict(success_rate=.90,collisions=0),
            passed=bool(np.mean([r['success'] for r in test_rows])>=.90 and sum(r['collisions'] for r in test_rows)==0))
        (dest/'test_report.json').write_text(json.dumps(report,indent=2))
        with (dest/'test_episodes.csv').open('w') as f:
            writer=csv.DictWriter(f,fieldnames=['policy']+list(test_rows[0])); writer.writeheader()
            for name,rows in [('baseline',test_baseline),('learned',test_rows)]:
                for row in rows: writer.writerow(dict(policy=name,**row))
        print(json.dumps(dict(event='final_test',**report)),flush=True)

if __name__=='__main__': main()
