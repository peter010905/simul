"""Audit LiDAR approximation using training scenes, never the consumed test set."""

from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import shutil
import time

import yaml

from evaluate import task,summarize
from osp_mission.surface_endpoint_policy import load_policy


def main():
    out=Path('training_robust/lidar_ablation');out.mkdir(exist_ok=False)
    snapshot=out/'source_snapshot';snapshot.mkdir()
    for source in Path('src/osp_mission/osp_mission').glob('*.py'):
        shutil.copy2(source,snapshot/source.name)
    for source in [Path(__file__),Path(__file__).parent/'evaluate.py']:
        shutil.copy2(source,snapshot/source.name)
    cfgs=[yaml.safe_load(p.read_text()) for p in sorted(Path('scenarios_robust/train_diverse').glob('*.yaml'))[:40]]
    weights=load_policy('src/osp_mission/config/surface_endpoint_policy.json')
    reference=Path('/opt/ros/jazzy/share/turtlebot4_description/urdf/sensors/rplidar.urdf.xacro')
    shutil.copy2(reference,out/'rplidar_reference.xacro')
    arms=[(120,False),(120,True),(640,False),(640,True)]
    metadata=dict(environment='2d_surrogate',split='train_diverse_first40',test_loaded=False,validation_loaded=False,
        seeds=[c['seed'] for c in cfgs],parameters=weights.tolist(),camera_width=960,camera_hz=5,
        source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.glob('*.py')},arms=arms,
        limitations=['20 cm raster','instantaneous rather than rolling scan','LiDAR at base pose without the -4 cm URDF offset','no range noise or SLAM pose error'])
    (out/'configuration.json').write_text(json.dumps(metadata,indent=2))
    results=[]
    with ProcessPoolExecutor(max_workers=6) as pool:
        for samples,body in arms:
            start=time.monotonic();settings=dict(lidar_samples=samples,lidar_body_frame=body)
            rows=list(pool.map(task,[(cfg,'surface_endpoint',weights,5.,3,settings) for cfg in cfgs]))
            stats=summarize(rows);result=dict(settings=settings,summary=stats,rows=rows,wall_seconds=time.monotonic()-start)
            (out/f'{samples}_{"body" if body else "fixed"}.json').write_text(json.dumps(result,indent=2))
            results.append(result)
            print(json.dumps(dict(settings=settings,success=stats['success_rate'],
                worst=min(v['success_rate'] for v in stats['by_mount'].values()),seconds=stats['mean_seconds'],
                contacts=stats['collisions'],wall_seconds=result['wall_seconds'])),flush=True)
    (out/'completed.json').write_text(json.dumps(dict(episodes=160,unique_training_layouts=40,test_loaded=False,
        results=[{k:v for k,v in r.items() if k!='rows'} for r in results]),indent=2))


if __name__=='__main__':main()
