"""Validate frozen candidates at the ROS SLAM grid scale on development scenes."""

import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import shutil
import time

import yaml

from evaluate import task,summarize


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    parser.add_argument('--planner',required=True);parser.add_argument('--policy',required=True)
    parser.add_argument('--workers',type=int,default=6);args=parser.parse_args()
    out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    snapshot=out/'source_snapshot';snapshot.mkdir()
    for source in Path('src/osp_mission/osp_mission').iterdir():
        if source.suffix in ('.py','.cpp','.so'):shutil.copy2(source,snapshot/source.name)
    for source in [Path(__file__),Path(__file__).parent/'evaluate.py']:shutil.copy2(source,snapshot/source.name)
    shutil.copy2(args.policy,out/'policy.json')
    policy=json.loads((out/'policy.json').read_text())
    if policy['planner']!=args.planner:raise ValueError('Planner and policy differ')
    cfgs=[yaml.safe_load(p.read_text()) for p in sorted(Path('scenarios_robust/v2/validation').glob('*.yaml'))]
    settings=dict(lidar_samples=640,lidar_body_frame=True,raycast_backend='native',continuous_shortcuts=True,resolution=.05)
    metadata=dict(environment='2d_surrogate',split='v2/validation',test_loaded=False,
        seeds=[c['seed'] for c in cfgs],parameters=policy['parameters'],planner=args.planner,camera_width=960,camera_hz=5,settings=settings,
        resolution_reason='src/osp_mission/config/slam.yaml specifies 0.05 m; not selected for best score',
        source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.iterdir() if p.is_file()},
        scenario_hash=hashlib.sha256(json.dumps(cfgs,sort_keys=True).encode()).hexdigest())
    (out/'configuration.json').write_text(json.dumps(metadata,indent=2))
    started=time.monotonic();rows=[]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        work=[(c,args.planner,policy['parameters'],5.,3,dict(settings,frame_quarter_turns=(c['seed']//20)%4)) for c in cfgs]
        for row in pool.map(task,work):
            row['frame_quarter_turns']=(row['seed']//20)%4;rows.append(row)
            if len(rows)%20==0:print(json.dumps(dict(completed=len(rows),total=len(cfgs))),flush=True)
    stats=summarize(rows);stats['mean_safety_stops']=sum(r['safety_stops'] for r in rows)/len(rows)
    stats['by_frame']={str(frame):summarize([r for r in rows if r['frame_quarter_turns']==frame]) for frame in range(4)}
    report=dict(environment='2d_surrogate',planner=args.planner,settings=settings,summary=stats,rows=rows,
        test_loaded=False,gazebo_verified=False,wall_seconds=time.monotonic()-started)
    (out/'completed.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(dict(success=stats['success_rate'],worst=min(v['success_rate'] for v in stats['by_mount'].values()),
        seconds=stats['mean_seconds'],contacts=stats['collisions'],mean_stops=stats['mean_safety_stops'])),flush=True)


if __name__=='__main__':main()
