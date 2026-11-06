"""Check observation-distance settings against the same held-out validation set."""

from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import shutil
import time

import numpy as np
import yaml

from evaluate import task,summarize


def main():
    out=Path('training_robust/standoff_probe');out.mkdir(exist_ok=False)
    snapshot=out/'source_snapshot';snapshot.mkdir()
    for source in Path('src/osp_mission/osp_mission').glob('*.py'):shutil.copy2(source,snapshot/source.name)
    shutil.copy2(Path(__file__).parent/'evaluate.py',snapshot/'evaluate.py')
    cfgs=[yaml.safe_load(p.read_text()) for p in sorted(Path('scenarios_robust/validation').glob('*.yaml'))]
    weights=np.array(json.loads(Path('src/osp_mission/config/surface_endpoint_policy.json').read_text())['parameters'])
    metadata=dict(planner='surface_endpoint',parameters=weights.tolist(),split='validation',test_evaluated=False,
        camera_hz=5,camera_width=960,source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.glob('*.py')},
        scenario_hash=hashlib.sha256(json.dumps(cfgs,sort_keys=True).encode()).hexdigest())
    with ProcessPoolExecutor(max_workers=4) as pool:
        for metres in [1.2,1.6,2.4]:
            start=time.monotonic();settings=dict(view_standoff=metres)
            rows=list(pool.map(task,[(c,'surface_endpoint',weights,5.,3,settings) for c in cfgs]));stats=summarize(rows)
            result=dict(metadata,view_standoff=metres,summary=stats,rows=rows,wall_seconds=time.monotonic()-start)
            (out/f'{metres:.1f}.json').write_text(json.dumps(result,indent=2))
            print(json.dumps(dict(metres=metres,success=stats['success_rate'],worst=min(v['success_rate'] for v in stats['by_mount'].values()),collisions=stats['collisions'],approached=stats['mean_approached'])),flush=True)
    (out/'completed.json').write_text(json.dumps(dict(test_evaluated=False,validation_episodes=240)))


if __name__=='__main__':main()
