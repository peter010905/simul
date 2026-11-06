"""Screen physical-face memory with common calibrated LiDAR on training scenes."""

from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import shutil
import time

import yaml

from evaluate import task,summarize


def main():
    out=Path('training_robust/edge_ablation');out.mkdir(exist_ok=False)
    snapshot=out/'source_snapshot';snapshot.mkdir()
    for source in Path('src/osp_mission/osp_mission').glob('*.py'):shutil.copy2(source,snapshot/source.name)
    for source in [Path(__file__),Path(__file__).parent/'evaluate.py']:shutil.copy2(source,snapshot/source.name)
    cfgs=[yaml.safe_load(p.read_text()) for p in sorted(Path('scenarios_robust/train_diverse').glob('*.yaml'))[:80]]
    arms=[]
    for planner,filename in [('surface_endpoint','surface_endpoint_policy.json'),('directional','directional_policy.json'),('edge','surface_endpoint_policy.json')]:
        arms.append(dict(planner=planner,parameters=json.loads((Path('src/osp_mission/config')/filename).read_text())['parameters']))
    settings=dict(lidar_samples=640,lidar_body_frame=True)
    metadata=dict(environment='2d_surrogate',split='train_diverse_first80',test_loaded=False,validation_loaded=False,
        seeds=[c['seed'] for c in cfgs],arms=arms,camera_width=960,camera_hz=5,settings=settings,
        source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.glob('*.py')})
    (out/'configuration.json').write_text(json.dumps(metadata,indent=2))
    results=[]
    with ProcessPoolExecutor(max_workers=6) as pool:
        for arm in arms:
            start=time.monotonic()
            rows=list(pool.map(task,[(cfg,arm['planner'],arm['parameters'],5.,3,settings) for cfg in cfgs]))
            stats=summarize(rows);result=dict(arm=arm,summary=stats,rows=rows,wall_seconds=time.monotonic()-start)
            (out/f"{arm['planner']}.json").write_text(json.dumps(result,indent=2));results.append(result)
            print(json.dumps(dict(planner=arm['planner'],success=stats['success_rate'],
                worst=min(v['success_rate'] for v in stats['by_mount'].values()),seconds=stats['mean_seconds'],
                contacts=stats['collisions'],wall_seconds=result['wall_seconds'])),flush=True)
    (out/'completed.json').write_text(json.dumps(dict(episodes=240,unique_training_layouts=80,test_loaded=False,
        results=[{k:v for k,v in r.items() if k!='rows'} for r in results]),indent=2))


if __name__=='__main__':main()
