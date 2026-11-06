"""Prove local normal updates preserve entire fine-grid search trajectories."""

from concurrent.futures import ProcessPoolExecutor
import hashlib
import importlib.util
import json
from pathlib import Path
import time

import numpy as np
import yaml

from osp_mission import edge_policy
from osp_mission.fast_sim import SearchEnv


def run(seed):
    reference=Path('training_robust/fidelity_validation/edge/source_snapshot/edge_policy.py')
    spec=importlib.util.spec_from_file_location('prior_edge_policy',reference)
    prior=importlib.util.module_from_spec(spec);spec.loader.exec_module(prior)
    cfg=yaml.safe_load(Path(f'scenarios_robust/train_diverse/{seed}.yaml').read_text())
    weights=json.loads(Path('src/osp_mission/config/edge_policy.json').read_text())['parameters']
    settings=dict(seed=seed+123456,planner='edge',resolution=.05,camera_width=960,camera_hz=5,
        lidar_samples=640,lidar_body_frame=True,raycast_backend='native',continuous_shortcuts=True,
        detection_dropout=cfg['sensor_stress']['detection_dropout'])
    current_class=edge_policy.EdgeCoverage
    try:
        edge_policy.EdgeCoverage=prior.EdgeCoverage
        old=SearchEnv(cfg,**settings)
    finally:edge_policy.EdgeCoverage=current_class
    new=SearchEnv(cfg,**settings)
    old.speed=min(old.speed,cfg['sensor_stress']['speed_limit']);new.speed=old.speed
    start=time.monotonic();before=old.run(weights);old_seconds=time.monotonic()-start
    start=time.monotonic();after=new.run(weights);new_seconds=time.monotonic()-start
    assert before==after,(before,after)
    for a,b in [(old.trace,new.trace),(old.map,new.map),(old.viewed,new.viewed),(old.edge_coverage.counts,new.edge_coverage.counts)]:
        np.testing.assert_array_equal(a,b)
    return dict(seed=seed,identical_metrics=True,identical_trace=True,identical_map=True,identical_coverage_counts=True,
        old_wall_seconds=old_seconds,new_wall_seconds=new_seconds,speedup=old_seconds/new_seconds,
        full_normal_updates=new.edge_coverage.normal_cache.full_updates,local_normal_updates=new.edge_coverage.normal_cache.local_updates,
        trace_sha256=hashlib.sha256(np.asarray(new.trace).tobytes()).hexdigest())


def main():
    with ProcessPoolExecutor(max_workers=2) as pool:rows=list(pool.map(run,[8100003,8100019]))
    report=dict(passed=True,test_loaded=False,rows=rows,source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
        for p in Path('src/osp_mission/osp_mission').iterdir() if p.suffix in ('.py','.cpp','.so')})
    Path('training_robust/normal_cache_equivalence.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='source_hashes'}),flush=True)


if __name__=='__main__':main()
