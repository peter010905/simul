"""Freeze balanced training scenes and observation-execution comparisons."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

import yaml


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    parser.add_argument('--seed-start',type=int,default=8500000);parser.add_argument('--count',type=int,default=80)
    args=parser.parse_args();out=Path(args.output);out.mkdir(exist_ok=False)
    cfgs=[yaml.safe_load(Path(f'scenarios_robust/v2/train/{seed}.yaml').read_text()) for seed in range(args.seed_start,args.seed_start+args.count)]
    snap=out/'source_snapshot';snap.mkdir()
    for source in Path('src/osp_mission/osp_mission').iterdir():
        if source.suffix in ('.py','.cpp','.so'):shutil.copy2(source,snap/source.name)
    shutil.copy2('training_robust/evaluate.py',snap/'evaluate.py');shutil.copy2(__file__,snap/'driver.py')
    parameters=json.loads(Path('src/osp_mission/config/surface_endpoint_policy.json').read_text())['parameters']
    settings=dict(resolution=.05,lidar_samples=640,lidar_body_frame=True,raycast_backend='native',continuous_shortcuts=True,collect_motion_timing=True)
    metadata=dict(environment='2d_surrogate',split='v2/train',parameters=parameters,settings=settings,
        seeds=[c['seed'] for c in cfgs],test_loaded=False,validation_loaded=False,
        camera_width=960,camera_hz=5,deadline=600,
        scenario_hash=hashlib.sha256(json.dumps(cfgs,sort_keys=True).encode()).hexdigest(),
        source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snap.iterdir() if p.is_file()})
    variants=[]
    for planner in ('surface_endpoint','symmetric_edge'):
        for optimized in (False,True):
            variants.append([planner+('_efficient' if optimized else '_baseline'),planner,
                             dict(settings,optimize_scan_alignment=optimized)])
    (out/'configuration.json').write_text(json.dumps(metadata,indent=2))
    (out/'variants.json').write_text(json.dumps(variants,indent=2))
    print(json.dumps(dict(prepared=len(cfgs),variants=len(variants),test_loaded=False)))


if __name__=='__main__':main()
