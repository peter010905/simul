"""Predeclared comparison on previously unused geometries; no policy selection loop."""

from concurrent.futures import ProcessPoolExecutor
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import time

import yaml

from evaluate import task, summarize
from osp_mission.curriculum import validate_split_separation
from osp_mission.surface_endpoint_policy import load_configuration


def wilson(successes,total):
    z=1.95996398454;p=successes/total;denominator=1+z*z/total
    centre=(p+z*z/(2*total))/denominator
    half=z*math.sqrt(p*(1-p)/total+z*z/(4*total*total))/denominator
    return [centre-half,centre+half]


def main():
    out=Path('training_robust/frozen_generalization_v1');out.mkdir(exist_ok=False)
    snapshot=out/'source_snapshot';snapshot.mkdir()
    for source in Path('src/osp_mission/osp_mission').glob('*.py'):
        shutil.copy2(source,snapshot/source.name)
    for source in [Path(__file__),Path(__file__).parent/'evaluate.py']:
        shutil.copy2(source,snapshot/source.name)
    arms=[]
    for name,filename in [('previous','surface_endpoint_policy.json'),('candidate','surface_wide_policy.json')]:
        source=Path('src/osp_mission/config')/filename
        frozen=out/f'{name}_policy.json';shutil.copy2(source,frozen)
        weights,settings=load_configuration(frozen)
        arms.append(dict(name=name,parameters=weights.tolist(),settings=settings,
            policy_sha256=hashlib.sha256(frozen.read_bytes()).hexdigest()))
    # Freeze the exact policies and acceptance rule before loading test scenes.
    registration=dict(frozen_utc=datetime.now(timezone.utc).isoformat(),environment='2d_surrogate',
        arms=arms,split='scenarios_robust/test',frame_quarter_turns=[0,1],camera_width=960,camera_hz=5,
        deadline=600,acceptance='candidate success >= previous in both frames; no contacts; combined mean_found >= previous',
        strong_goal='at least 90% success in each frame, no contacts; Gazebo validation still separately required',
        source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.glob('*.py')})
    (out/'registration.json').write_text(json.dumps(registration,indent=2))
    cfgs=[yaml.safe_load(p.read_text()) for p in sorted(Path(registration['split']).glob('*.yaml'))]
    development=[yaml.safe_load(p.read_text()) for directory in ['train_diverse','validation']
                 for p in sorted((Path('scenarios_robust')/directory).glob('*.yaml'))]
    validate_split_separation(development,cfgs)
    (out/'test_manifest.json').write_text(json.dumps(dict(unique_layouts=len(cfgs),seeds=[c['seed'] for c in cfgs],
        scenario_hash=hashlib.sha256(json.dumps(cfgs,sort_keys=True).encode()).hexdigest()),indent=2))
    results={}
    with ProcessPoolExecutor(max_workers=6) as pool:
        for arm in arms:
            for frame in registration['frame_quarter_turns']:
                start=time.monotonic()
                settings=dict(view_standoff=arm['settings']['view_standoff'],frame_quarter_turns=frame)
                planner='surface_deadline' if arm['settings']['deadline_aware'] else 'surface_endpoint'
                rows=list(pool.map(task,[(cfg,planner,arm['parameters'],5.,3,settings) for cfg in cfgs]))
                stats=summarize(rows);stats['success_wilson_95']=wilson(sum(r['success'] for r in rows),len(rows))
                result=dict(name=arm['name'],frame_quarter_turns=frame,summary=stats,rows=rows,wall_seconds=time.monotonic()-start)
                (out/f"{arm['name']}_frame{frame}.json").write_text(json.dumps(result,indent=2))
                results[(arm['name'],frame)]=result
                print(json.dumps(dict(name=arm['name'],frame=frame,success=stats['success_rate'],
                    worst=min(v['success_rate'] for v in stats['by_mount'].values()),seconds=stats['mean_seconds'],
                    collisions=stats['collisions'])),flush=True)
    paired=[]
    for frame in registration['frame_quarter_turns']:
        before=results['previous',frame]['rows'];after=results['candidate',frame]['rows']
        assert [r['seed'] for r in before]==[r['seed'] for r in after]
        paired.append(dict(frame=frame,gained=sum(not a['success'] and b['success'] for a,b in zip(before,after)),
            lost=sum(a['success'] and not b['success'] for a,b in zip(before,after)),
            both_success=sum(a['success'] and b['success'] for a,b in zip(before,after))))
    candidate=[results['candidate',frame]['summary'] for frame in [0,1]]
    previous=[results['previous',frame]['summary'] for frame in [0,1]]
    accepted=all(a['success_rate']>=b['success_rate'] and a['collisions']==0 for a,b in zip(candidate,previous)) and sum(a['mean_found'] for a in candidate)>=sum(b['mean_found'] for b in previous)
    report=dict(independent_test_evaluated=True,unique_test_layouts=len(cfgs),episodes=4*len(cfgs),
        same_geometries_repeated_across_frames=True,paired=paired,accepted=accepted,
        strong_2d_threshold_met=all(s['success_rate']>=.9 and s['collisions']==0 for s in candidate),
        gazebo_verified=False,results=[{k:v for k,v in result.items() if k!='rows'} for result in results.values()])
    (out/'completed.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='results'}),flush=True)


if __name__=='__main__':main()
