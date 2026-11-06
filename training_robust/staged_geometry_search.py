"""Bounded weight learning on two disjoint, balanced training batches."""

import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import importlib
import json
from pathlib import Path
import shutil
import time

import numpy as np
import yaml

from evaluate import task, summarize
from osp_mission.curriculum import LayoutCurriculum


def rank(stats):
    worst = min(v['success_rate'] for v in stats['by_mount'].values())
    return (-stats['collisions'], .8 * stats['success_rate'] + .2 * worst,
            stats['mean_found'], -stats['mean_seconds'])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--planner', default='symmetric_edge')
    parser.add_argument('--initial-policy', default='training_robust/symmetric_candidate.json')
    parser.add_argument('--workers', type=int, default=10)
    parser.add_argument('--seed', type=int, default=2026110583)
    args = parser.parse_args()
    out = Path(args.output); out.mkdir(parents=True, exist_ok=False)
    snapshot = out / 'source_snapshot'; snapshot.mkdir()
    package = snapshot / 'osp_mission'; package.mkdir()
    for source in Path(importlib.import_module('osp_mission').__file__).parent.iterdir():
        if source.suffix in ('.py', '.cpp', '.so'):
            shutil.copy2(source, package / source.name)
    shutil.copy2(__file__, snapshot / 'driver.py')
    shutil.copy2(Path(__file__).parent / 'evaluate.py', snapshot / 'evaluate.py')
    initial = json.loads(Path(args.initial_policy).read_text())['parameters']
    rng = np.random.default_rng(args.seed)
    proposals = np.exp(np.clip(rng.normal(np.log(initial), .65, (5, 5)), -3, 3))
    proposals[:, 0] = 1.
    population = [initial] + proposals.tolist()
    configs = [yaml.safe_load(p.read_text()) for p in sorted(Path('scenarios_robust/v2/train').glob('*.yaml'))]
    curriculum = LayoutCurriculum(configs, args.seed + 1)
    batches = {}
    for stage_index, stage in enumerate(('stage1', 'stage2')):
        batch = curriculum.next_batch() + curriculum.next_batch()
        batches[stage] = [(cfg, (index % 20 + index // 20 + 2 * stage_index) % 4)
                          for index, cfg in enumerate(batch)]
    seeds = [cfg['seed'] for batch in batches.values() for cfg, _ in batch]
    assert len(seeds) == len(set(seeds)) == 80
    settings = dict(resolution=.05, lidar_samples=640, lidar_body_frame=True,
                    raycast_backend='native', continuous_shortcuts=True)
    config = dict(environment='2d_surrogate', planner=args.planner, parameters=population,
                  settings=settings, camera_width=960, camera_hz=5, seed=args.seed,
                  search='initial plus five log-normal proposals on forty balanced layouts; top two and initial on forty different layouts',
                  schedule={stage: [dict(seed=cfg['seed'], frame=frame) for cfg, frame in batch]
                            for stage, batch in batches.items()},
                  training_pool=len(configs), test_loaded=False, validation_loaded=False,
                  source_hashes={str(p.relative_to(snapshot)): hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in snapshot.rglob('*') if p.is_file()},
                  scenario_hash=hashlib.sha256(json.dumps(configs, sort_keys=True).encode()).hexdigest())
    (out / 'configuration.json').write_text(json.dumps(config, indent=2))
    started = time.monotonic(); episodes = 0
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        def evaluate(name, stage, planner, weights):
            nonlocal episodes
            work = [(cfg, planner, weights, 5., 3, dict(settings, frame_quarter_turns=frame))
                    for cfg, frame in batches[stage]]
            rows = []
            for row, (_, frame) in zip(pool.map(task, work), batches[stage]):
                row['frame_quarter_turns'] = frame
                rows.append(row)
                if len(rows) % 10 == 0:
                    print(json.dumps(dict(name=name, completed=len(rows), total=len(work))), flush=True)
                    (out / 'partial.json').write_text(json.dumps(dict(name=name, rows=rows)))
            stats = summarize(rows); episodes += len(rows)
            result = dict(name=name, stage=stage, planner=planner, parameters=weights, summary=stats, rows=rows)
            (out / f'{name}.json').write_text(json.dumps(result, indent=2))
            print(json.dumps(dict(name=name, success=stats['success_rate'],
                                  worst=min(v['success_rate'] for v in stats['by_mount'].values()),
                                  seconds=stats['mean_seconds'], contacts=stats['collisions'])), flush=True)
            return result
        stage1 = [evaluate(f'proposal_{i}', 'stage1', args.planner, w) for i, w in enumerate(population)]
        elites = sorted(range(len(population)), key=lambda i: rank(stage1[i]['summary']), reverse=True)[:2]
        if 0 not in elites:
            elites.append(0)
        (out / 'selected_for_stage2.json').write_text(json.dumps(dict(indices=elites, training_episodes=episodes), indent=2))
        stage2 = [evaluate(f'elite_{i}', 'stage2', args.planner, population[i]) for i in elites]
        control = evaluate('endpoint_control', 'stage2', 'surface_endpoint', initial)
    best = max(stage2, key=lambda result: rank(result['summary']))
    candidate = dict(planner=args.planner, parameters=best['parameters'], sensor_settings=settings,
                     training_stage2=best['summary'], training_episodes=episodes,
                     status='experimental_training_search_gazebo_unverified', default=False)
    (out / 'best_policy.json').write_text(json.dumps(candidate, indent=2))
    report = dict(completed=True, training_episodes=episodes, unique_training_layouts=80,
                  validation_loaded=False, test_loaded=False, best_name=best['name'],
                  best_training=best['summary'], endpoint_control=control['summary'],
                  better_than_endpoint_on_stage2=rank(best['summary']) > rank(control['summary']),
                  adopted=False, wall_seconds=time.monotonic() - started)
    (out / 'completed.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k not in ('best_training', 'endpoint_control')}), flush=True)


if __name__ == '__main__':
    main()
