"""Compare fixed candidates under identical timing and both map-frame directions."""

from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import shutil
import time

import numpy as np
import yaml

from evaluate import task, summarize


def main():
    out=Path('training_robust/distance_budget_v5');out.mkdir(exist_ok=False)
    snapshot=out/'source_snapshot';snapshot.mkdir()
    for source in Path('src/osp_mission/osp_mission').glob('*.py'):
        shutil.copy2(source,snapshot/source.name)
    shutil.copy2(Path(__file__).parent/'evaluate.py',snapshot/'evaluate.py')
    shutil.copy2(__file__,snapshot/Path(__file__).name)
    cfgs=[yaml.safe_load(p.read_text()) for p in sorted(Path('scenarios_robust/validation').glob('*.yaml'))]
    weights=np.array(json.loads(Path('src/osp_mission/config/surface_endpoint_policy.json').read_text())['parameters'])
    arms=[('default','surface_endpoint',2.),('wide','surface_endpoint',2.4),
          ('wide_deadline','surface_deadline',2.4)]
    metadata=dict(environment='2d_surrogate',test_evaluated=False,parameters=weights.tolist(),
        camera_width=960,camera_hz=5,deadline=600,
        source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.glob('*.py')},
        scenario_hash=hashlib.sha256(json.dumps(cfgs,sort_keys=True).encode()).hexdigest(),
        arms=arms,frame_quarter_turns=[0,1])
    (out/'configuration.json').write_text(json.dumps(metadata,indent=2))
    results=[]
    with ProcessPoolExecutor(max_workers=6) as pool:
        for name,planner,standoff in arms:
            for frame in [0,1]:
                started=time.monotonic()
                settings=dict(view_standoff=standoff,frame_quarter_turns=frame)
                rows=list(pool.map(task,[(cfg,planner,weights,5.,3,settings) for cfg in cfgs]))
                stats=summarize(rows)
                result=dict(name=name,planner=planner,settings=settings,summary=stats,rows=rows,
                            wall_seconds=time.monotonic()-started)
                (out/f'{name}_frame{frame}.json').write_text(json.dumps(result,indent=2))
                results.append(result)
                print(json.dumps(dict(name=name,frame=frame,success=stats['success_rate'],
                    worst=min(s['success_rate'] for s in stats['by_mount'].values()),
                    seconds=stats['mean_seconds'],collisions=stats['collisions'])),flush=True)
    (out/'completed.json').write_text(json.dumps(dict(test_evaluated=False,episodes=480,
        results=[{key:value for key,value in result.items() if key!='rows'} for result in results]),indent=2))


if __name__=='__main__':main()
