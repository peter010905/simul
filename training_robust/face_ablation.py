"""Screen surface-driven viewpoint changes on a balanced TRAINING batch."""

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
    out=Path('training_robust/face_ablation_v1');out.mkdir(exist_ok=False)
    snapshot=out/'source_snapshot';snapshot.mkdir()
    for p in Path('src/osp_mission/osp_mission').glob('*.py'):
        shutil.copy2(p,snapshot/p.name)
    shutil.copy2(__file__,out/'driver.py')
    cfgs=[yaml.safe_load(Path(f'scenarios_robust/train_diverse/{8400000+i}.yaml').read_text()) for i in range(20)]
    w=np.array(json.loads(Path('training_robust/diverse_initial.json').read_text())['parameters'])
    endpoint=np.array(json.loads(Path('training_robust/endpoint_cem/best_policy.json').read_text())['parameters'])
    variants=[('directional','directional',w),('face_candidates','face_candidates',w),
              ('object_balanced','object_balanced',w),('face_object','face_object',w),
              ('face_object_endpoint_weights','face_object',endpoint),
              ('object_balanced_endpoint_weights','object_balanced',endpoint)]
    metadata=dict(split='train_diverse',seeds=[c['seed'] for c in cfgs],camera_hz=5,
                  test_evaluated=False,source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.glob('*.py')})
    (out/'configuration.json').write_text(json.dumps(metadata,indent=2))
    with ProcessPoolExecutor(max_workers=4) as pool:
        for name,planner,weights in variants:
            start=time.monotonic();rows=list(pool.map(task,[(c,planner,weights,5.) for c in cfgs]));s=summarize(rows)
            (out/f'{name}.json').write_text(json.dumps(dict(metadata,name=name,planner=planner,parameters=weights.tolist(),summary=s,rows=rows,wall_seconds=time.monotonic()-start),indent=2))
            print(json.dumps(dict(name=name,success=s['success_rate'],worst=min(v['success_rate'] for v in s['by_mount'].values()),collisions=s['collisions'],time=s['mean_seconds'])),flush=True)
    (out/'completed.json').write_text(json.dumps(dict(training_episodes=120,test_evaluated=False)))


if __name__=='__main__':main()
