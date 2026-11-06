"""Run the current mission on balanced Gazebo layouts, recording startup failures separately."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

import yaml

from osp_mission.curriculum import LayoutCurriculum
from osp_mission.gazebo_eval import one


def select_scenarios(directory, seed, limit):
    paths=sorted(Path(directory).glob('*.yaml'))
    cfgs=[yaml.safe_load(p.read_text()) for p in paths]
    if not 1<=limit<=len(cfgs):raise ValueError('Limit must fit available unique layouts')
    curriculum=LayoutCurriculum(cfgs,seed)
    selected=[];seen=set();by_seed={c['seed']:p for c,p in zip(cfgs,paths)}
    while len(selected)<limit:
        for cfg in curriculum.next_batch():
            if cfg['seed'] in seen:continue
            seen.add(cfg['seed']);selected.append((by_seed[cfg['seed']],cfg))
            if len(selected)==limit:break
    return selected


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--scenarios',default='scenarios_robust/validation')
    parser.add_argument('--policy',default='src/osp_mission/config/surface_endpoint_policy.json')
    parser.add_argument('--output',required=True)
    parser.add_argument('--limit',type=int,default=20)
    parser.add_argument('--seed',type=int,default=2026110553)
    parser.add_argument('--domain',type=int,default=84)
    parser.add_argument('--wall-timeout',type=float,default=5400.)
    parser.add_argument('--prepare-only',action='store_true')
    parser.add_argument('--run-prepared',action='store_true')
    parser.add_argument('--record-video',action='store_true')
    args=parser.parse_args()
    out=Path(args.output).resolve()
    if not args.run_prepared:
        out.mkdir(parents=True,exist_ok=False)
        selected=select_scenarios(args.scenarios,args.seed,args.limit)
        shutil.copy2(args.policy,out/'policy.json')
        scenes=out/'scenarios';scenes.mkdir()
        entries=[]
        for path,cfg in selected:
            frozen=scenes/path.name;shutil.copy2(path,frozen)
            entries.append(dict(file=str(frozen),seed=cfg['seed'],mount_profile=cfg['mount_profile'],
                requested_obstacles=cfg['difficulty'],actual_obstacles=len(cfg['obstacles']),
                sha256=hashlib.sha256(frozen.read_bytes()).hexdigest()))
        manifest=dict(environment='gazebo_harmonic',planner=json.loads((out/'policy.json').read_text())['planner'],camera_width=960,duration=600,render_backend='software',start_docked=False,approach_radius_m=1.5,protocol_version=2,
            policy_sha256=hashlib.sha256((out/'policy.json').read_bytes()).hexdigest(),scenarios=entries)
        (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    else:
        manifest=json.loads((out/'manifest.json').read_text())
    if args.prepare_only:
        print(json.dumps(dict(prepared=len(manifest['scenarios']),output=str(out),gazebo_run=False)),flush=True)
        return
    if (out/'report.json').exists():raise ValueError('This benchmark was already attempted; preserve its results')
    if hashlib.sha256((out/'policy.json').read_bytes()).hexdigest()!=manifest['policy_sha256']:
        raise ValueError('Frozen policy changed')
    rows=[]
    source=Path('src/osp_mission')
    snapshot=out/'source_snapshot';shutil.copytree(source,snapshot,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    source_hashes={str(p.relative_to(source)):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in source.rglob('*') if p.is_file() and not {'__pycache__','.pytest_cache'}.intersection(p.parts)}
    (out/'source_hashes.json').write_text(json.dumps(source_hashes,indent=2))
    for entry in manifest['scenarios']:
        if any(not (source/p).exists() or hashlib.sha256((source/p).read_bytes()).hexdigest()!=digest
               for p,digest in source_hashes.items()):raise RuntimeError('Runtime source changed during benchmark; preserve results and start a new run')
        if hashlib.sha256(Path(entry['file']).read_bytes()).hexdigest()!=entry['sha256']:
            raise ValueError('Frozen scenario changed')
        result=one(entry['file'],out/'policy.json',out/'episodes'/str(entry['seed']),
            domain=args.domain,duration=manifest['duration'],wall_timeout=args.wall_timeout,
            planner=manifest['planner'],camera_width=manifest['camera_width'],record_video=args.record_video,render_backend=manifest.get('render_backend','software'),start_docked=manifest.get('start_docked',False))
        rows.append(dict(scenario=entry,result=result))
        completed=[r for r in rows if r['result'].get('reason') in ('all_targets_found','timeout')]
        report=dict(environment='gazebo_harmonic',planned=len(manifest['scenarios']),attempted=len(rows),
            completed_missions=len(completed),successful_missions=sum(r['result']['success'] for r in completed),
            infrastructure_failures=sum(r['result'].get('reason') not in ('all_targets_found','timeout') for r in rows),
            complete=len(completed)==len(manifest['scenarios']),rows=rows,
            success_rate_over_attempted=sum(r['result']['success'] for r in rows)/len(rows),record_video=args.record_video)
        (out/'report.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(dict(seed=entry['seed'],reason=result.get('reason'),success=result['success'])),flush=True)
        if result.get('reason') in ('evaluator_startup_failure','startup_failure'):
            # Repeating failed launches cannot establish navigation performance.
            break


if __name__=='__main__':main()
