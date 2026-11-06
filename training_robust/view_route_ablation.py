"""Training-only paired screening of explicit views and short route lookahead."""

import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import shutil
import time

import yaml

from evaluate import task, summarize


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--workers', type=int, default=8)
    parser.add_argument('--count', type=int, default=40)
    parser.add_argument('--seed-start', type=int, default=8500000)
    parser.add_argument('--planners', nargs='+', default=['surface_endpoint', 'edge', 'view_greedy', 'view_route'])
    parser.add_argument('--glance-thresholds', type=float, nargs='*', default=[])
    args = parser.parse_args()
    if args.count not in (20, 40, 80):
        raise ValueError('Use a balanced multiple of 20 training layouts')
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    snapshot = out / 'source_snapshot'
    snapshot.mkdir()
    for source in Path('src/osp_mission/osp_mission').iterdir():
        if source.suffix in ('.py', '.cpp', '.so'):
            shutil.copy2(source, snapshot / source.name)
    shutil.copy2(__file__, snapshot / 'driver.py')
    shutil.copy2(Path(__file__).parent / 'evaluate.py', snapshot / 'evaluate.py')
    cfgs = [yaml.safe_load(Path(f'scenarios_robust/v2/train/{args.seed_start + i}.yaml').read_text()) for i in range(args.count)]
    weights = json.loads(Path('src/osp_mission/config/surface_endpoint_policy.json').read_text())['parameters']
    settings = dict(resolution=.05, lidar_samples=640, lidar_body_frame=True,
                    raycast_backend='native', continuous_shortcuts=True)
    meta = dict(environment='2d_surrogate', split='v2/train', test_loaded=False,
                validation_loaded=False, parameters=weights, settings=settings,
                seeds=[c['seed'] for c in cfgs], camera_width=960, camera_hz=5,
                scenario_hash=hashlib.sha256(json.dumps(cfgs, sort_keys=True).encode()).hexdigest(),
                source_hashes={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.iterdir()})
    (out / 'configuration.json').write_text(json.dumps(meta, indent=2))
    variants = [(planner, planner, settings) for planner in args.planners]
    if args.glance_thresholds:
        variants = [(f'glance_{threshold:g}', 'edge_glance', dict(settings, travel_glance_threshold=threshold))
                    for threshold in args.glance_thresholds]
    (out / 'variants.json').write_text(json.dumps(variants, indent=2))
    results = []
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for name, planner, variant_settings in variants:
            start = time.monotonic()
            work = [(c, planner, weights, 5., 3, dict(variant_settings, frame_quarter_turns=(c['seed'] // 20) % 4)) for c in cfgs]
            rows = []
            for row in pool.map(task, work):
                rows.append(row)
                if len(rows) % 10 == 0:
                    print(json.dumps(dict(name=name, planner=planner, completed=len(rows), total=len(cfgs))), flush=True)
            stats = summarize(rows)
            result = dict(name=name, planner=planner, settings=variant_settings, summary=stats, rows=rows, wall_seconds=time.monotonic() - start)
            (out / f'{name}.json').write_text(json.dumps(result, indent=2))
            results.append(result)
            print(json.dumps(dict(name=name, planner=planner, success=stats['success_rate'],
                                  worst=min(v['success_rate'] for v in stats['by_mount'].values()),
                                  seconds=stats['mean_seconds'], contacts=stats['collisions'],
                                  wall_seconds=result['wall_seconds'])), flush=True)
    (out / 'completed.json').write_text(json.dumps(dict(episodes=len(cfgs) * len(results),
                                                        unique_training_layouts=len(cfgs),
                                                        test_loaded=False, validation_loaded=False,
                                                        results=[{k: v for k, v in r.items() if k != 'rows'} for r in results]), indent=2))


if __name__ == '__main__':
    main()
