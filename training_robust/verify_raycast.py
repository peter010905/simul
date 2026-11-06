"""Compare complete trajectories against the frozen pre-acceleration model."""

from concurrent.futures import ProcessPoolExecutor
import hashlib
import ast
import importlib.util
import json
from pathlib import Path
import time

import numpy as np
import yaml

from osp_mission.fast_sim import SearchEnv


def run(args):
    seed,planner=args
    spec=importlib.util.spec_from_file_location('prior_fast_sim','training_robust/edge_ablation/source_snapshot/fast_sim.py')
    prior=importlib.util.module_from_spec(spec);spec.loader.exec_module(prior)
    cfg=yaml.safe_load(Path(f'scenarios_robust/train_diverse/{seed}.yaml').read_text())
    weights=json.loads(Path('src/osp_mission/config/surface_endpoint_policy.json').read_text())['parameters']
    settings=dict(seed=seed+123456,planner=planner,camera_width=960,camera_hz=5,
        detection_dropout=cfg['sensor_stress']['detection_dropout'],lidar_samples=640,lidar_body_frame=True)
    old=prior.SearchEnv(cfg,**settings);new=SearchEnv(cfg,raycast_backend='native',continuous_shortcuts=False,**settings)
    old.speed=min(old.speed,cfg['sensor_stress']['speed_limit']);new.speed=old.speed
    begin=time.monotonic();old_result=old.run(weights);old_seconds=time.monotonic()-begin
    begin=time.monotonic();new_result=new.run(weights);new_seconds=time.monotonic()-begin
    assert old_result==new_result,(old_result,new_result)
    for before,after in [(old.map,new.map),(old.viewed,new.viewed),(old.trace,new.trace)]:
        np.testing.assert_array_equal(before,after)
    return dict(seed=seed,planner=planner,identical_metrics=True,identical_trace=True,identical_map=True,
        identical_viewed_faces=True,old_wall_seconds=old_seconds,new_wall_seconds=new_seconds,
        speedup=old_seconds/new_seconds,trace_sha256=hashlib.sha256(np.asarray(new.trace).tobytes()).hexdigest())


def main():
    # Changes to these modules would invalidate the older-model comparison.
    for name in ['surface_endpoint_policy.py','edge_policy.py','directional_policy.py','surface_policy.py','policy.py','scenarios.py']:
        before=ast.parse((Path('training_robust/edge_ablation/source_snapshot')/name).read_text())
        after=ast.parse((Path('src/osp_mission/osp_mission')/name).read_text())
        # Loading a ROS policy file is outside this simulator comparison.
        for tree in (before,after):tree.body=[node for node in tree.body if not isinstance(node,ast.FunctionDef) or node.name!='load_policy']
        assert ast.dump(before)==ast.dump(after),name
    with ProcessPoolExecutor(max_workers=4) as pool:
        rows=list(pool.map(run,[(seed,planner) for seed in [8100003,8100019] for planner in ['surface_endpoint','edge']]))
    result=dict(passed=True,rows=rows,all_exact=True,test_loaded=False,continuous_shortcuts=False,
        source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in Path('src/osp_mission/osp_mission').iterdir() if p.suffix in ('.py','.cpp','.so')})
    Path('training_robust/raycast_equivalence_legacy_geometry.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)


if __name__=='__main__':main()
