"""Extend the training pool with fresh geometry; never read the held-out test."""

import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path

import yaml

from osp_mission.curriculum import LayoutCurriculum, validate_split_separation
from osp_mission.robust_scenarios import COUNTS, PROFILES, make_robust


def generate(args):
    return make_robust(*args)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='scenarios_robust/train_diverse')
    parser.add_argument('--start', type=int, default=8400000)
    parser.add_argument('--fresh-count', type=int, default=100)
    parser.add_argument('--workers', type=int, default=2)
    args = parser.parse_args()
    if args.fresh_count <= 0 or args.fresh_count % 20:
        raise ValueError('Fresh count must be a positive multiple of 20')
    destination = Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    if list(destination.iterdir()):
        raise ValueError('Use an empty output directory')
    original = [yaml.safe_load(p.read_text())
                for p in sorted(Path('scenarios_robust/train').glob('*.yaml'))]
    tasks = [(args.start + i, COUNTS[i % 5], PROFILES[(i // 5) % 4])
             for i in range(args.fresh_count)]
    fresh = []
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for config in pool.map(generate, tasks):
            fresh.append(config)
            if len(fresh) % 20 == 0:
                print(f'Generated {len(fresh)}/{args.fresh_count} new layouts', flush=True)
    configs = original + fresh
    validation = [yaml.safe_load(p.read_text())
                  for p in sorted(Path('scenarios_robust/validation').glob('*.yaml'))]
    validate_split_separation(configs, validation)
    schedule = LayoutCurriculum(configs, 2026110502)
    geometries = [hashlib.sha256(json.dumps(sorted(
        json.dumps(o, sort_keys=True) for o in c['obstacles'])).encode()).hexdigest()
        for c in configs]
    if len(set(geometries)) != len(configs):
        raise ValueError('Duplicate training geometry')
    for config in configs:
        (destination / f"{config['seed']}.yaml").write_text(
            yaml.safe_dump(config, sort_keys=False))
    manifest = dict(
        original_count=len(original), new_count=len(fresh),
        unique_geometries=len(set(geometries)),
        seeds=[c['seed'] for c in configs],
        strata=[dict(mount=g[0]['mount_profile'], obstacles=g[0]['difficulty'],
                     layouts=len(g)) for g in schedule.groups],
        all_layouts_seen_after_generations=max(map(len, schedule.groups)),
        validation_separate=True, test_loaded=False)
    (destination / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    print(json.dumps({k: v for k, v in manifest.items() if k not in ('seeds', 'strata')}), flush=True)


if __name__ == '__main__':
    main()
