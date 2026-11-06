"""Paired full-scene check of continuous shortcut clearance at three grid scales."""

from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import shutil
import time

import yaml

from evaluate import task,summarize


def main():
    out=Path('training_robust/path_resolution_corrected');out.mkdir(exist_ok=False)
    snapshot=out/'source_snapshot';snapshot.mkdir()
    for source in Path('src/osp_mission/osp_mission').iterdir():
        if source.suffix in ('.py','.cpp','.so'):shutil.copy2(source,snapshot/source.name)
    for source in [Path(__file__),Path(__file__).parent/'evaluate.py']:shutil.copy2(source,snapshot/source.name)
    cfgs=[yaml.safe_load(p.read_text()) for p in sorted(Path('scenarios_robust/train_diverse').glob('*.yaml'))[:20]]
    weights=json.loads(Path('src/osp_mission/config/surface_endpoint_policy.json').read_text())['parameters']
    settings=dict(lidar_samples=640,lidar_body_frame=True,raycast_backend='native',continuous_shortcuts=True)
    metadata=dict(environment='2d_surrogate',split='train_diverse_first20',test_loaded=False,validation_loaded=False,
        seeds=[c['seed'] for c in cfgs],parameters=weights,camera_width=960,camera_hz=5,settings=settings,
        reference='training_robust/resolution_ablation',
        source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.iterdir() if p.is_file()})
    (out/'configuration.json').write_text(json.dumps(metadata,indent=2))
    results=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for resolution in [.2,.1,.05]:
            for planner in ['surface_endpoint','edge']:
                start=time.monotonic();parameters=dict(settings,resolution=resolution)
                rows=list(pool.map(task,[(cfg,planner,weights,5.,3,parameters) for cfg in cfgs]))
                stats=summarize(rows);stats['mean_safety_stops']=sum(r['safety_stops'] for r in rows)/len(rows)
                result=dict(planner=planner,resolution=resolution,summary=stats,rows=rows,wall_seconds=time.monotonic()-start)
                (out/f'{planner}_{resolution}.json').write_text(json.dumps(result,indent=2));results.append(result)
                print(json.dumps(dict(planner=planner,resolution=resolution,success=stats['success_rate'],
                    worst=min(v['success_rate'] for v in stats['by_mount'].values()),seconds=stats['mean_seconds'],
                    contacts=stats['collisions'],mean_stops=stats['mean_safety_stops'],wall_seconds=result['wall_seconds'])),flush=True)
    (out/'completed.json').write_text(json.dumps(dict(episodes=120,unique_training_layouts=20,test_loaded=False,
        results=[{k:v for k,v in r.items() if k!='rows'} for r in results]),indent=2))


if __name__=='__main__':main()
