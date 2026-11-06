"""Broader development splits, with separately reserved unseen test seeds."""

from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path

import yaml

from osp_mission.curriculum import validate_split_separation
from osp_mission.robust_scenarios import COUNTS,PROFILES,make_robust


def generate(args):return make_robust(*args)


def main():
    root=Path('scenarios_robust/v2');root.mkdir(exist_ok=False)
    old=[yaml.safe_load(p.read_text()) for p in sorted(Path('scenarios_robust/train_diverse').glob('*.yaml'))]
    splits={}
    with ProcessPoolExecutor(max_workers=4) as pool:
        for split,start,count in [('train',8500000,200),('validation',8600000,160)]:
            configs=old.copy() if split=='train' else []
            for cfg in pool.map(generate,[(start+i,COUNTS[i%5],PROFILES[(i//5)%4]) for i in range(count)]):
                configs.append(cfg)
                if len(configs)%20==0:print(json.dumps(dict(split=split,generated=len(configs))),flush=True)
            splits[split]=configs
            destination=root/split;destination.mkdir()
            for cfg in configs:(destination/f"{cfg['seed']}.yaml").write_text(yaml.safe_dump(cfg,sort_keys=False))
    validate_split_separation(splits['train'],splits['validation'])
    manifest=dict(split_sizes={k:len(v) for k,v in splits.items()},seeds={k:[c['seed'] for c in v] for k,v in splits.items()},
        test_seeds_reserved=list(range(8700000,8700200)),test_generated=False,test_loaded=False,
        excluded_consumed_test_seeds=[8300000,8300099],
        difficulty_preserved=dict(obstacles=list(COUNTS),mount_profiles=list(PROFILES),marker_sizes=[.12,.30],deadline=600),
        hashes={k:hashlib.sha256(json.dumps(v,sort_keys=True).encode()).hexdigest() for k,v in splits.items()})
    (root/'manifest.json').write_text(json.dumps(manifest,indent=2))
    print(json.dumps(dict(completed=True,split_sizes=manifest['split_sizes'],test_generated=False)),flush=True)


if __name__=='__main__':main()
