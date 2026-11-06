"""Small frozen CEM pretraining study after native frontier diagnostics.

This is explicitly a 2D surrogate study, never evidence of Gazebo success.
The reserved native holdout is not read or used for candidate selection.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

import numpy as np
import yaml
from evaluate import task,summarize
from osp_mission.curriculum import LayoutCurriculum,validate_split_separation


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True)
    parser.add_argument('--frozen',action='store_true')
    args=parser.parse_args();out=Path(args.output).resolve()
    if not args.frozen:
        out.mkdir(parents=True,exist_ok=False)
        snap=out/'source_snapshot';snap.mkdir()
        shutil.copytree('src/osp_mission/osp_mission',snap/'osp_mission',ignore=shutil.ignore_patterns('__pycache__'))
        shutil.copy2(__file__,snap/'driver.py');shutil.copy2(Path(__file__).parent/'evaluate.py',snap/'evaluate.py')
        hashes={str(p.relative_to(snap)):hashlib.sha256(p.read_bytes()).hexdigest() for p in snap.rglob('*') if p.is_file()}
        (out/'source_hashes.json').write_text(json.dumps(hashes,indent=2))
        raise SystemExit(subprocess.call([sys.executable,str(snap/'driver.py'),'--frozen','--output',str(out)]))
    train=[yaml.safe_load(p.read_text()) for p in sorted(Path('scenarios_robust/v2/train').glob('*.yaml'))]
    valid=[yaml.safe_load(p.read_text()) for p in sorted(Path('scenarios_robust/v2/validation').glob('*.yaml'))]
    validate_split_separation(train,valid)
    curriculum=LayoutCurriculum(train,2026110588)
    validation_curriculum=LayoutCurriculum(valid,2026110589)
    validation=validation_curriculum.next_batch()+validation_curriculum.next_batch()
    champion=np.array([1.,1.204765992697077,.39116097662487975,.7812959734881743,.6473764656964455])
    rng=np.random.default_rng(2026110590);mean=np.log(champion);std=np.full(5,.8)
    settings=dict(resolution=.1,lidar_samples=640,lidar_body_frame=True,raycast_backend='native',
                  camera_width=960,angular_profile='trapezoid',max_turn_speed=.55,max_turn_acceleration=1.2,
                  continuous_shortcuts=True)
    config=dict(environment='2d_surrogate',purpose='candidate pretraining only; native evaluation required',
                generations=2,population=6,workers=2,settings=settings,validation_seeds=[c['seed'] for c in validation],
                reserved_native_holdout_loaded=False)
    (out/'configuration.json').write_text(json.dumps(config,indent=2))
    history=[];start=time.monotonic()
    def score(stats):
        return (-stats['collisions'],.8*stats['success_rate']+.2*min(v['success_rate'] for v in stats['by_mount'].values()),stats['mean_found'],-stats['mean_seconds'])
    def work(cfgs,weights,generation):
        return [(cfg,'surface_endpoint',weights,5.,3,dict(settings,frame_quarter_turns=(cfg['seed']+generation)%4)) for cfg in cfgs]
    with ProcessPoolExecutor(max_workers=2) as pool:
        def evaluate(cfgs,weights,generation,label):
            rows=[]
            for row in pool.map(task,work(cfgs,weights,generation)):
                rows.append(row)
                if len(rows)%10==0:print(json.dumps(dict(phase=label,completed=len(rows),total=len(cfgs))),flush=True)
            stats=summarize(rows)
            (out/f'{label}.json').write_text(json.dumps(dict(parameters=weights.tolist(),summary=stats,rows=rows),indent=2))
            return stats
        best=evaluate(validation,champion,0,'initial_validation')
        for generation in range(2):
            batch=curriculum.next_batch()
            population=np.exp(np.clip(rng.normal(mean,std,(6,5)),-3,3));population[:,0]=1.;population[0]=champion
            if generation==0:population[1]=champion*np.array([1.,4.,1.,1.,1.])
            trials=[]
            for i,weights in enumerate(population):
                trials.append(evaluate(batch,weights,generation,f'train_{generation}_{i}'))
            indices=sorted(range(6),key=lambda i:score(trials[i]),reverse=True)
            elite=np.log(population[indices[:2]]);mean=.4*mean+.6*elite.mean(axis=0);std=np.maximum(.2,.5*std+.5*elite.std(axis=0))
            candidate=population[indices[0]]
            candidate_stats=best if np.array_equal(candidate,champion) else evaluate(validation,candidate,0,f'validation_{generation}')
            if score(candidate_stats)>score(best):champion=candidate.copy();best=candidate_stats
            history.append(dict(generation=generation,seeds=[c['seed'] for c in batch],parameters=population.tolist(),training=trials,best_validation=best))
            (out/'history.json').write_text(json.dumps(history,indent=2))
            (out/'candidate_policy.json').write_text(json.dumps(dict(planner='surface_endpoint',parameters=champion.tolist(),
                status='surrogate_pretrained_native_unverified',training_episodes=(generation+1)*120,validation=best),indent=2))
            print(json.dumps(dict(generation=generation,validation_success=best['success_rate'],wall_seconds=time.monotonic()-start)),flush=True)
    (out/'completed.json').write_text(json.dumps(dict(training_episodes=240,unique_training_layouts=40,
        validation_layouts=40,environment='2d_surrogate',reserved_native_holdout_loaded=False,wall_seconds=time.monotonic()-start),indent=2))


if __name__=='__main__':main()
