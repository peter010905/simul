"""Compare frozen candidates using the same corrected sensor and shield model."""

from concurrent.futures import ProcessPoolExecutor
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import time

import numpy as np
import yaml

from evaluate import summarize, task


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',default='training_robust/timing_v3')
    args=parser.parse_args()
    root = Path(args.output)
    root.mkdir(parents=True, exist_ok=False)
    snapshot = root / 'source_snapshot'
    snapshot.mkdir()
    for path in Path('src/osp_mission/osp_mission').glob('*.py'):
        shutil.copy2(path, snapshot / path.name)
    shutil.copy2(Path(__file__).parent / 'evaluate.py', snapshot / 'evaluate.py')
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in snapshot.glob('*.py')}
    configs = [yaml.safe_load(p.read_text())
               for p in sorted(Path('scenarios_robust/validation').glob('*.yaml'))]
    sources = [
        ('legacy', 'src/osp_mission/config/assignment_policy.json'),
        ('surface_endpoint', 'training_robust/endpoint_cem/best_policy.json'),
        ('directional', 'training_robust/diverse_initial.json')]
    policies = []
    for planner, path in sources:
        policy = json.loads(Path(path).read_text())
        (root / f'{planner}_frozen.json').write_text(json.dumps(policy, indent=2))
        # Legacy policies have their own parameter vector schema.
        if planner == 'legacy':
            from osp_mission.policy import load_policy
            weights = load_policy(path)
        else:
            weights = np.array(policy['parameters'])
        policies.append((planner, weights))
    metadata = dict(source_hashes=hashes, environment='2d_surrogate',
                    camera_width=960, camera_hz=5, deadline=600,
                    split='validation', test_evaluated=False,
                    scenario_hash=hashlib.sha256(json.dumps(
                        configs, sort_keys=True).encode()).hexdigest())
    (root / 'configuration.json').write_text(json.dumps(metadata, indent=2))
    with ProcessPoolExecutor(max_workers=4) as pool:
        for planner, weights in policies:
            start = time.monotonic()
            rows = list(pool.map(task, [(c, planner, weights, 5.) for c in configs]))
            report = dict(metadata, planner=planner, parameters=weights.tolist(),
                          summary=summarize(rows), rows=rows,
                          wall_seconds=time.monotonic() - start)
            (root / f'{planner}_validation80.json').write_text(json.dumps(report, indent=2))
            print(json.dumps(dict(planner=planner, summary=report['summary'])), flush=True)
    (root / 'completed.json').write_text(json.dumps(dict(test_evaluated=False)))


if __name__ == '__main__':
    main()
