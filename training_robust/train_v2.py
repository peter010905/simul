"""Balanced 400-layout CEM with map-frame augmentation and frozen resume sources."""

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

from evaluate import task,summarize
from osp_mission.curriculum import LayoutCurriculum,validate_split_separation


def rank(stats):
    worst=min(v['success_rate'] for v in stats['by_mount'].values())
    return (-stats['collisions'],.8*stats['success_rate']+.2*worst,stats['mean_found'],-stats['mean_seconds'])


def training_score(stats):
    worst=min(v['success_rate'] for v in stats['by_mount'].values())
    return 10000*stats['success_rate']+2500*worst+100*stats['mean_found']-.15*stats['mean_seconds']-100000*stats['collisions']


def batch_with_frames(curriculum,generation):
    # Four unique layouts in each of 20 strata, one in each map frame.
    return [(cfg,(repetition+generation)%4) for repetition in range(4) for cfg in curriculum.next_batch()]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True)
    parser.add_argument('--planner',default='edge')
    parser.add_argument('--initial-policy',default='src/osp_mission/config/surface_endpoint_policy.json')
    parser.add_argument('--seed',type=int,default=2026110561)
    parser.add_argument('--workers',type=int,default=6)
    parser.add_argument('--generation-limit',type=int,default=2)
    parser.add_argument('--resolution',type=float,default=.05)
    parser.add_argument('--resume',action='store_true')
    args=parser.parse_args()
    if args.generation_limit<1:raise ValueError('Use a positive finite generation limit')
    if not np.isfinite(args.resolution) or not .025<=args.resolution<=.25:
        raise ValueError('Grid resolution must be between 0.025 and 0.25 metres')
    out=Path(args.output).resolve()
    package=Path(importlib.import_module('osp_mission').__file__).parent
    if args.resume:
        configuration=json.loads((out/'configuration.json').read_text())
        if package.resolve()!=(out/'source_snapshot/osp_mission').resolve():
            raise ValueError('Resume through recover_v2.py to use the frozen simulator')
        for name,digest in configuration['source_hashes'].items():
            if hashlib.sha256((out/'source_snapshot'/name).read_bytes()).hexdigest()!=digest:
                raise ValueError(f'Frozen source changed: {name}')
    else:
        out.mkdir(parents=True,exist_ok=False)
        snapshot=out/'source_snapshot';snapshot.mkdir()
        frozen=snapshot/'osp_mission';frozen.mkdir()
        for source in package.iterdir():
            if source.suffix in ('.py','.cpp','.so'):shutil.copy2(source,frozen/source.name)
        shutil.copy2(__file__,snapshot/'driver.py')
        shutil.copy2(Path(__file__).parent/'evaluate.py',snapshot/'evaluate.py')
        configuration=dict(planner=args.planner,seed=args.seed,generations=5,population=8,batch=80,
            train_dir='scenarios_robust/v2/train',validation_dir='scenarios_robust/v2/validation',
            lidar_samples=640,lidar_body_frame=True,raycast_backend='native',camera_hz=5,camera_width=960,deadline=600,
            resolution=args.resolution,continuous_shortcuts=True,
            initial_parameters=json.loads(Path(args.initial_policy).read_text())['parameters'],
            sampling='stratified_without_replacement_four_frames_per_stratum',
            validation_frame='(seed // 20) % 4, two layouts per stratum per frame',
            selection='zero contacts first, 0.8 success + 0.2 worst mount; one training elite validated per generation',
            reserved_test_seed_range=[8700000,8700199],test_loaded=False,
            source_hashes={str(p.relative_to(snapshot)):hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.rglob('*') if p.is_file()})
    cfgs={split:[yaml.safe_load(p.read_text()) for p in sorted(Path(configuration[split+'_dir']).glob('*.yaml'))] for split in ['train','validation']}
    validate_split_separation(cfgs['train'],cfgs['validation'])
    if len(cfgs['train'])!=400 or len(cfgs['validation'])!=160:raise ValueError('Incomplete v2 splits')
    hashes={split:hashlib.sha256(json.dumps(cs,sort_keys=True).encode()).hexdigest() for split,cs in cfgs.items()}
    if args.resume:
        if hashes!=configuration['scenario_hashes']:raise ValueError('Scenario data changed')
    else:
        configuration['scenario_hashes']=hashes
        (out/'configuration.json').write_text(json.dumps(configuration,indent=2))
    rng=np.random.default_rng(configuration['seed'])
    curriculum=LayoutCurriculum(cfgs['train'],configuration['seed']+1)
    champion=np.asarray(configuration['initial_parameters'],dtype=float)
    mean=np.log(champion);std=np.full(5,.65);history=[];best=None;first_generation=0;elapsed_before=0.
    if args.resume:
        with np.load(out/'checkpoint.npz') as checkpoint:
            first_generation=int(checkpoint['generation'])+1
            champion=checkpoint['champion'].copy();mean=checkpoint['mean'].copy();std=checkpoint['std'].copy()
            rng.bit_generator.state=json.loads(str(checkpoint['rng_state']))
            curriculum.restore(json.loads(str(checkpoint['curriculum_state'])))
        history=json.loads((out/'history.json').read_text())[:first_generation]
        if len(history)!=first_generation:raise ValueError('Checkpoint history is incomplete')
        best=history[-1]['best_validation'];elapsed_before=history[-1]['wall_seconds']
    settings=dict(lidar_samples=configuration['lidar_samples'],lidar_body_frame=configuration['lidar_body_frame'],
        raycast_backend=configuration['raycast_backend'],resolution=configuration['resolution'],
        continuous_shortcuts=configuration['continuous_shortcuts'],camera_width=configuration['camera_width'])
    def tasks(batch,weights):
        return [(cfg,configuration['planner'],weights,5.,3,dict(settings,frame_quarter_turns=frame)) for cfg,frame in batch]
    validation=[(cfg,(cfg['seed']//20)%4) for cfg in cfgs['validation']]
    start=time.monotonic()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        def collect(work,phase):
            rows=[]
            for result in pool.map(task,work):
                rows.append(result)
                if len(rows)%80==0:
                    print(json.dumps(dict(phase=phase,completed=len(rows),total=len(work))),flush=True)
            return rows
        if best is None:
            rows=collect(tasks(validation,champion),'initial_validation')
            best=summarize(rows)
            (out/'initial_validation.json').write_text(json.dumps(dict(summary=best,rows=rows),indent=2))
            print(json.dumps(dict(initial_validation=best['success_rate'],worst=min(s['success_rate'] for s in best['by_mount'].values()))),flush=True)
        end=min(configuration['generations'],first_generation+args.generation_limit)
        for generation in range(first_generation,end):
            batch=batch_with_frames(curriculum,generation)
            schedule=[dict(seed=cfg['seed'],frame=frame) for cfg,frame in batch]
            (out/'current_batch.json').write_text(json.dumps(dict(generation=generation,scenarios=schedule),indent=2))
            population=np.exp(np.clip(rng.normal(mean,std,(configuration['population'],5)),-3,3))
            population[:,0]=1.;population[0]=champion
            rows=collect([item for weights in population for item in tasks(batch,weights)],f'train_{generation}')
            training=[summarize(rows[i*80:(i+1)*80]) for i in range(len(population))]
            scores=np.array([training_score(stats) for stats in training])
            elites=np.log(population[np.argsort(scores)[-3:]])
            mean=.4*mean+.6*elites.mean(axis=0);std=np.maximum(.2,.5*std+.5*elites.std(axis=0))
            index=int(np.argmax(scores));candidate=population[index]
            stats=best if np.array_equal(candidate,champion) else summarize(collect(tasks(validation,candidate),f'validation_{generation}'))
            if rank(stats)>rank(best):best=stats;champion=candidate.copy()
            history.append(dict(generation=generation,schedule=schedule,training_episodes=(generation+1)*640,
                training_candidates=[dict(parameters=w.tolist(),score=float(s),summary=t) for w,s,t in zip(population,scores,training)],
                selected_index=index,candidate_validation=stats,best_validation=best,wall_seconds=elapsed_before+time.monotonic()-start))
            (out/'history.json').write_text(json.dumps(history,indent=2))
            (out/'best_policy.json').write_text(json.dumps(dict(planner=configuration['planner'],parameters=champion.tolist(),
                sensor_settings=settings,validation=best,training_episodes=(generation+1)*640,status='experimental_v2_gazebo_unverified'),indent=2))
            with (out/'checkpoint.tmp').open('wb') as stream:
                np.savez(stream,generation=generation,champion=champion,mean=mean,std=std,
                    rng_state=json.dumps(rng.bit_generator.state),curriculum_state=json.dumps(curriculum.state()))
            (out/'checkpoint.tmp').replace(out/'checkpoint.npz')
            print(json.dumps(dict(generation=generation,training_episodes=(generation+1)*640,
                success=best['success_rate'],worst=min(s['success_rate'] for s in best['by_mount'].values()),
                contacts=best['collisions'],wall_seconds=history[-1]['wall_seconds'])),flush=True)
    complete=len(history)==configuration['generations']
    marker='completed.json' if complete else f'chunk_finished_{len(history):03d}.json'
    used=[r['seed'] for generation in history for r in generation['schedule']]
    (out/marker).write_text(json.dumps(dict(completed=complete,next_generation=len(history),
        training_episodes=len(history)*640,unique_training_layouts=len(set(used)),test_loaded=False),indent=2))


if __name__=='__main__':main()
