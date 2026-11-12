"""Verify actual occupancy variation separately from seeds and marker identity."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/osp_mission'))
from osp_mission.scenarios import rasterize


def occupancy_hash(scene):
    grid = rasterize(scene, resolution=.10)
    return hashlib.sha256(grid.tobytes()).hexdigest()


def audit(campaign):
    state = json.loads((campaign / 'state.json').read_text())
    rows = []
    for job in state['jobs']:
        if job['phase'] != 'train':
            continue
        path = Path(job['scenario']['file'])
        actual_hash = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual_hash != job['scenario']['sha256']:
            raise ValueError(f'Scenario file changed after preparation: {path}')
        scene = yaml.safe_load(path.read_text())
        rows.append(dict(job=job['number'], generation=job['generation'], seed=scene['seed'],
            requested_obstacles=scene['difficulty'], actual_obstacles=len(scene['obstacles']),
            mount_profile=scene['mount_profile'], start=scene['start'],
            marker_sizes_m=[target['size'] for target in scene['targets']],
            occupancy_sha256=occupancy_hash(scene)))
    unique = len({row['occupancy_sha256'] for row in rows})
    positions = np.array([row['start'][:2] for row in rows])
    return dict(evidence='Scenario collision/support geometry rasterized at0.10m; this is not an observed SLAM map or mission outcome',
        training_episodes=len(rows), unique_occupancy_maps=unique,
        no_reused_occupancy=unique == len(rows),
        requested_obstacle_counts=sorted({row['requested_obstacles'] for row in rows}),
        mount_profiles=sorted({row['mount_profile'] for row in rows}),
        distinct_start_positions=len({tuple(row['start'][:2]) for row in rows}),
        start_xy_min_m=positions.min(axis=0).tolist() if len(rows) else [],
        start_xy_max_m=positions.max(axis=0).tolist() if len(rows) else [],
        boundary_m=[20.15, 11.35], rows=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('campaign', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.campaign)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({key: value for key, value in result.items() if key != 'rows'}))
    if not result['training_episodes'] or not result['no_reused_occupancy']:
        raise SystemExit('Training occupancy is empty or reused.')


if __name__ == '__main__':
    main()
