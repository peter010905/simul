"""Bounded staged policy search on 60 training geometries, with frozen sources."""

import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import importlib
import json
from pathlib import Path
import shutil
import time

import numpy as np
import yaml

from evaluate import task, summarize


def rank(stats):
    return (-stats['collisions'], .8 * stats['success_rate'] + .2 * min(v['success_rate'] for v in stats['by_mount'].values()),
            stats['mean_found'], -stats['mean_seconds'])


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    parser.add_argument('--workers',type=int,default=8)
    args=parser.parse_args()
    out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    snapshot=out/'source_snapshot';snapshot.mkdir();package=snapshot/'osp_mission';package.mkdir()
    for source in Path(importlib.import_module('osp_mission').__file__).parent.iterdir():
        if source.suffix in ('.py','.cpp','.so'):shutil.copy2(source,package/source.name)
    shutil.copy2(__file__,snapshot/'driver.py');shutil.copy2(Path(__file__).parent/'evaluate.py',snapshot/'evaluate.py')
    initial=json.loads(Path('src/osp_mission/config/surface_endpoint_policy.json').read_text())['parameters']
    nominal=np.array([1.,1.,1.,1.,.25]);rng=np.random.default_rng(2026110573)
    population=[initial,nominal.tolist(),[1.,2.,1.,.3,1.]]
    random=np.exp(rng.normal(np.log(nominal),.7,(3,5)));random[:,0]=1.
    population.extend(random.tolist())
    scenes={name:[yaml.safe_load(Path(f'scenarios_robust/v2/train/{seed}.yaml').read_text()) for seed in seeds]
            for name,seeds in [('stage1',range(8500040,8500060)),('stage2',range(8500060,8500100))]}
    settings=dict(resolution=.05,lidar_samples=640,lidar_body_frame=True,raycast_backend='native',continuous_shortcuts=True)
    config=dict(environment='2d_surrogate',planner='view_route',camera_width=960,camera_hz=5,settings=settings,
                search='three starting policies plus three log-normal proposals; top two reevaluated on forty different training layouts',
                parameters=population,seed=2026110573,train_seeds={k:[c['seed'] for c in v] for k,v in scenes.items()},
                frame_quarter_turns='(seed // 20) % 4',test_loaded=False,validation_loaded=False,
                source_hashes={str(p.relative_to(snapshot)):hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.rglob('*') if p.is_file()},
                scenario_hash=hashlib.sha256(json.dumps(scenes,sort_keys=True).encode()).hexdigest())
    (out/'configuration.json').write_text(json.dumps(config,indent=2))
    started=time.monotonic();episodes=0
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        def evaluate(name,split,planner,weights):
            nonlocal episodes
            work=[(cfg,planner,weights,5.,3,dict(settings,frame_quarter_turns=(cfg['seed']//20)%4)) for cfg in scenes[split]]
            rows=[]
            for result in pool.map(task,work):
                rows.append(result)
                if len(rows)%10==0:print(json.dumps(dict(name=name,completed=len(rows),total=len(work))),flush=True)
            stats=summarize(rows);episodes+=len(rows)
            result=dict(name=name,split=split,planner=planner,parameters=weights,summary=stats,rows=rows)
            (out/f'{name}.json').write_text(json.dumps(result,indent=2))
            print(json.dumps(dict(name=name,success=stats['success_rate'],worst=min(v['success_rate'] for v in stats['by_mount'].values()),seconds=stats['mean_seconds'],contacts=stats['collisions'])),flush=True)
            return result
        stage1=[evaluate(f'proposal_{i}','stage1','view_route',w) for i,w in enumerate(population)]
        elites=sorted(range(len(population)),key=lambda i:rank(stage1[i]['summary']),reverse=True)[:2]
        (out/'selected_for_stage2.json').write_text(json.dumps(dict(indices=elites,training_episodes=episodes),indent=2))
        stage2=[evaluate(f'elite_{i}','stage2','view_route',population[i]) for i in elites]
        if 0 not in elites:stage2.append(evaluate('warm_control','stage2','view_route',initial))
        control=evaluate('endpoint_control','stage2','surface_endpoint',initial)
    best=max(stage2,key=lambda result:rank(result['summary']))
    candidate=dict(planner='view_route',parameters=best['parameters'],sensor_settings=settings,
                   training_stage2=best['summary'],training_episodes=episodes,
                   status='experimental_training_search_gazebo_unverified',default=False)
    (out/'best_policy.json').write_text(json.dumps(candidate,indent=2))
    report=dict(completed=True,training_episodes=episodes,unique_training_layouts=60,validation_loaded=False,test_loaded=False,
                best_name=best['name'],best_training=best['summary'],endpoint_control=control['summary'],
                better_than_endpoint_on_stage2=rank(best['summary'])>rank(control['summary']),
                adopted=False,wall_seconds=time.monotonic()-started)
    (out/'completed.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k not in ('best_training','endpoint_control')}),flush=True)


if __name__=='__main__':main()
