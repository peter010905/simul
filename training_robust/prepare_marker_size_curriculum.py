"""Generate new development scenes with independent, reproducible marker sizes."""
import argparse
import hashlib
import json
from pathlib import Path

import yaml
from osp_mission.marker_sizes import randomize_sizes
from osp_mission.robust_scenarios import make_robust,COUNTS,PROFILES
from osp_mission.scenarios import valid


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--minimum',type=float,default=.08)
    parser.add_argument('--maximum',type=float,default=.30)
    parser.add_argument('--size-mode',choices=['mixed','uniform'],default='mixed')
    parser.add_argument('--seed',type=int,default=9810000)
    parser.add_argument('--size-seed',type=int,default=19810000)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    manifest=dict(protocol='independent_marker_sizes_v1',minimum_m=args.minimum,maximum_m=args.maximum,
                  size_mode=args.size_mode,heldout_reserved_scenes_used=False,splits={})
    for split,offset,n in [('train',0,40),('validation',1000,20)]:
        directory=args.output/split;directory.mkdir();entries=[]
        for i in range(n):
            seed=args.seed+offset+i;size_seed=args.size_seed+offset+i
            cfg=randomize_sizes(make_robust(seed,COUNTS[i%5],PROFILES[(i//5)%4]),size_seed,args.minimum,args.maximum,args.size_mode)
            if not valid(cfg):raise ValueError(f'Resized marker backing made layout inaccessible: {seed}; choose another development seed')
            file=directory/f'{seed}.yaml';file.write_text(yaml.safe_dump(cfg,sort_keys=False))
            entries.append(dict(seed=seed,file=str(file.resolve()),requested_obstacles=cfg['difficulty'],
                actual_obstacles=len(cfg['obstacles']),mount_profile=cfg['mount_profile'],
                sizes_m=[t['size'] for t in cfg['targets']],size_seed=size_seed,sha256=hashlib.sha256(file.read_bytes()).hexdigest()))
        manifest['splits'][split]=entries
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    sizes=[s for rows in manifest['splits'].values() for row in rows for s in row['sizes_m']]
    print(json.dumps(dict(scenes=sum(len(v) for v in manifest['splits'].values()),markers=len(sizes),minimum_actual_m=min(sizes),maximum_actual_m=max(sizes))))


if __name__=='__main__':main()
