"""Resume frozen geometry experiments in finite, individually saved batches."""

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path
import shutil
import sys
import time

import yaml


def rank(stats):
    return (-stats['collisions'], .8 * stats['success_rate'] + .2 * min(v['success_rate'] for v in stats['by_mount'].values()),
            stats['mean_found'], -stats['mean_seconds'])


def atomic(path, data):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(data, indent=2)); temp.replace(path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', required=True)
    parser.add_argument('--workers', type=int, default=10)
    parser.add_argument('--max-episodes', type=int, default=20)
    args = parser.parse_args()
    out = Path(args.run).resolve(); snap = out / 'source_snapshot'
    config = json.loads((out / 'configuration.json').read_text())
    for name, digest in config['source_hashes'].items():
        if hashlib.sha256((snap / name).read_bytes()).hexdigest() != digest:
            raise ValueError(f'Frozen source changed: {name}')
    staged = 'schedule' in config
    if staged:
        package_parent = snap
    else:
        package_parent = out / 'resume_package'; package = package_parent / 'osp_mission'
        package.mkdir(parents=True, exist_ok=True)
        for source in snap.iterdir():
            if source.is_file() and source.name not in ('driver.py', 'evaluate.py'):
                shutil.copy2(source, package / source.name)
    sys.path.insert(0, str(snap)); sys.path.insert(0, str(package_parent))
    from evaluate import task, summarize
    from osp_mission import fast_sim
    if Path(fast_sim.__file__).resolve() != (package_parent / 'osp_mission/fast_sim.py').resolve():
        raise ValueError('Recovery imported an unfrozen simulator')
    if Path(sys.modules[task.__module__].__file__).resolve() != (snap / 'evaluate.py').resolve():
        raise ValueError('Recovery imported an unfrozen evaluator')
    configs = {int(p.stem): yaml.safe_load(p.read_text()) for p in Path('scenarios_robust/v2/train').glob('*.yaml')}
    if staged:
        expected_hash = hashlib.sha256(json.dumps([configs[k] for k in sorted(configs)], sort_keys=True).encode()).hexdigest()
    else:
        expected_hash = hashlib.sha256(json.dumps([configs[k] for k in config['seeds']], sort_keys=True).encode()).hexdigest()
    if expected_hash != config['scenario_hash']:
        raise ValueError('Training scenarios changed')
    selected = None
    if staged:
        candidates = [(f'proposal_{i}', 'stage1', config['planner'], weights)
                      for i, weights in enumerate(config['parameters'])]
        if all((out / f'{name}.json').exists() for name, *_ in candidates):
            first = [json.loads((out / f'{name}.json').read_text()) for name, *_ in candidates]
            elites = sorted(range(len(first)), key=lambda i: rank(first[i]['summary']), reverse=True)[:2]
            if 0 not in elites: elites.append(0)
            atomic(out / 'selected_for_stage2.json', dict(indices=elites, training_episodes=240))
            candidates = [(f'elite_{i}', 'stage2', config['planner'], config['parameters'][i]) for i in elites]
            candidates.append(('endpoint_control', 'stage2', 'surface_endpoint', config['parameters'][0]))
        selected = next((entry for entry in candidates if not (out / f'{entry[0]}.json').exists()), None)
        if selected:
            name, stage, planner, weights = selected
            schedule = config['schedule'][stage]
            settings = config['settings']
    else:
        variants = json.loads((out / 'variants.json').read_text())
        selected = next((entry for entry in variants if not (out / f'{entry[0]}.json').exists()), None)
        if selected:
            name, planner, settings = selected; stage = 'train'; weights = config['parameters']
            schedule = [dict(seed=s, frame=(s // 20) % 4) for s in config['seeds']]
    if selected:
        partial = out / f'{name}.partial.json'
        saved = json.loads(partial.read_text()) if partial.exists() else dict(rows={}, wall_seconds=0.)
        pending = [(index, entry) for index, entry in enumerate(schedule) if str(index) not in saved['rows']][:args.max_episodes]
        started = time.monotonic()
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            futures = {pool.submit(task, (configs[entry['seed']], planner, weights, 5., 3,
                                         dict(settings, frame_quarter_turns=entry['frame']))): index
                       for index, entry in pending}
            for future in as_completed(futures):
                index = futures[future]; row = future.result()
                if staged: row['frame_quarter_turns'] = schedule[index]['frame']
                saved['rows'][str(index)] = row
                atomic(partial, saved)
                print(json.dumps(dict(candidate=name, saved=len(saved['rows']), total=len(schedule))), flush=True)
        saved['wall_seconds'] += time.monotonic() - started; atomic(partial, saved)
        if len(saved['rows']) == len(schedule):
            rows = [saved['rows'][str(i)] for i in range(len(schedule))]; stats = summarize(rows)
            result = dict(name=name, stage=stage, planner=planner, parameters=weights, settings=settings,
                          summary=stats, rows=rows, wall_seconds=saved['wall_seconds'])
            atomic(out / f'{name}.json', result)
            print(json.dumps(dict(candidate=name, success=stats['success_rate'], worst=min(v['success_rate'] for v in stats['by_mount'].values()), seconds=stats['mean_seconds'], contacts=stats['collisions'])), flush=True)
        return
    if staged:
        results = [json.loads(p.read_text()) for p in sorted(out.glob('elite_*.json')) if '.partial.' not in p.name]
        best = max(results, key=lambda r: rank(r['summary']))
        control = json.loads((out / 'endpoint_control.json').read_text())
        episodes = 240 + 40 * (len(results) + 1)
        atomic(out / 'best_policy.json', dict(planner=config['planner'], parameters=best['parameters'],
               sensor_settings=config['settings'], training_stage2=best['summary'], training_episodes=episodes,
               status='experimental_training_search_gazebo_unverified', default=False))
        report = dict(completed=True, training_episodes=episodes, unique_training_layouts=80,
                      validation_loaded=False, test_loaded=False, best_name=best['name'],
                      best_training=best['summary'], endpoint_control=control['summary'],
                      better_than_endpoint_on_stage2=rank(best['summary']) > rank(control['summary']), adopted=False)
    else:
        results = [json.loads((out / f'{entry[0]}.json').read_text()) for entry in variants]
        report = dict(episodes=sum(len(r['rows']) for r in results), unique_training_layouts=len(config['seeds']),
                      test_loaded=False, validation_loaded=False,
                      results=[{k: v for k, v in r.items() if k != 'rows'} for r in results])
    report['resumed_from_frozen_sources'] = True
    atomic(out / 'completed.json', report)
    print(json.dumps(dict(completed=True, output=str(out))), flush=True)


if __name__ == '__main__':
    main()
