"""Resolution ablation on training layouts; no validation or test selection here."""

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
    out=Path('training_robust/camera_probe');out.mkdir(exist_ok=False)
    snapshot=out/'source_snapshot';snapshot.mkdir()
    for source in Path('src/osp_mission/osp_mission').glob('*.py'):
        shutil.copy2(source,snapshot/source.name)
    shutil.copy2(Path(__file__).parent/'evaluate.py',snapshot/'evaluate.py')
    shutil.copy2(__file__,snapshot/Path(__file__).name)
    cfgs=[yaml.safe_load(p.read_text()) for p in sorted(Path('scenarios_robust/train_diverse').glob('*.yaml'))[:80]]
    weights=np.array(json.loads(Path('src/osp_mission/config/surface_endpoint_policy.json').read_text())['parameters'])
    metadata=dict(environment='2d_surrogate',test_evaluated=False,validation_evaluated=False,
        parameters=weights.tolist(),source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.glob('*.py')},
        seeds=[c['seed'] for c in cfgs],camera_hz=5,view_standoff=2.4,deadline=600,widths=[960,1280,1600])
    (out/'configuration.json').write_text(json.dumps(metadata,indent=2))
    with ProcessPoolExecutor(max_workers=4) as pool:
        for width in metadata['widths']:
            start=time.monotonic();settings=dict(camera_width=width,view_standoff=2.4)
            rows=list(pool.map(task,[(cfg,'surface_endpoint',weights,5.,3,settings) for cfg in cfgs]))
            stats=summarize(rows)
            (out/f'{width}.json').write_text(json.dumps(dict(camera_width=width,summary=stats,rows=rows,
                wall_seconds=time.monotonic()-start),indent=2))
            print(json.dumps(dict(camera_width=width,success=stats['success_rate'],
                worst=min(v['success_rate'] for v in stats['by_mount'].values()),
                approached=stats['mean_approached'],seconds=stats['mean_seconds'],collisions=stats['collisions'])),flush=True)
    (out/'completed.json').write_text(json.dumps(dict(episodes=240,test_evaluated=False)))


if __name__=='__main__':main()
