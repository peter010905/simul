"""Evaluate coordinate-frame robustness without changing physical layouts."""

import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import yaml

from evaluate import summarize
from osp_mission.fast_sim import SearchEnv


def episode(arguments):
    cfg,planner,weights,turns=arguments
    stress=cfg.get('sensor_stress',{})
    env=SearchEnv(cfg,seed=cfg['seed']+123456,planner=planner,camera_width=960,
                  detection_dropout=stress.get('detection_dropout',.08),camera_hz=5,
                  frame_quarter_turns=turns)
    env.speed=min(env.speed,stress.get('speed_limit',.26))
    result=env.run(weights);trace=np.asarray(env.trace)
    trace=trace[trace[:,3]<=600]
    nearest=[float(np.hypot(trace[:,0]-t['x'],trace[:,1]-t['y']).min())
             for t in env.cfg['targets']]
    result.update(mount_profile=cfg['mount_profile'],actual_obstacles=len(cfg['obstacles']),
                  approached=sum(d<=2 for d in nearest),frame_quarter_turns=turns)
    return result


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--planner',default='surface_endpoint')
    parser.add_argument('--weights',default='src/osp_mission/config/surface_endpoint_policy.json')
    parser.add_argument('--quarter-turns',type=int,default=1,choices=[0,1,2,3])
    parser.add_argument('--workers',type=int,default=4)
    parser.add_argument('--output',required=True)
    args=parser.parse_args();out=Path(args.output)
    if out.exists():raise ValueError('Use a new result filename')
    configs=[yaml.safe_load(p.read_text()) for p in sorted(Path('scenarios_robust/validation').glob('*.yaml'))]
    weights=np.array(json.loads(Path(args.weights).read_text())['parameters'])
    source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in Path('src/osp_mission/osp_mission').glob('*.py')}
    start=time.monotonic()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows=list(pool.map(episode,[(c,args.planner,weights,args.quarter_turns) for c in configs]))
    result=dict(environment='2d_surrogate',split='validation',test_evaluated=False,
                frame_quarter_turns=args.quarter_turns,physical_layouts_changed=False,
                planner=args.planner,parameters=weights.tolist(),source_hashes=source_hashes,
                scenario_hash=hashlib.sha256(json.dumps(configs,sort_keys=True).encode()).hexdigest(),
                summary=summarize(rows),rows=rows,wall_seconds=time.monotonic()-start)
    out.write_text(json.dumps(result,indent=2));print(json.dumps(result['summary']),flush=True)


if __name__=='__main__':main()
