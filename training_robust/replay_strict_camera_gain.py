"""Retained-endpoint reward replay on finalized native approach-regression inputs."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np

from continual_native_curriculum import digest, save, source_hashes


def replay(episode, baseline, candidate, calibration):
    if not (episode/'evaluation.json').is_file():
        raise ValueError('Require a finalized native evaluation before replaying this regression')
    sys.path.insert(0, str(baseline/'src/osp_mission'))
    before, after = source_hashes(baseline), source_hashes(candidate)
    changed = {k for k in before.keys()|after.keys() if before.get(k) != after.get(k)}
    if changed != {'osp_mission/edge_policy.py', 'osp_mission/mission.py',
            'test/test_bounded_gain_runtime.py', 'test/test_calibrated_gain_runtime.py'}:
        raise ValueError('Require identical dependencies except the reviewed camera-gain changes')
    camera = json.loads(calibration.read_text())
    if not camera['native_identity_verified_after_observation']: raise ValueError('Require bound native calibration')
    modules = []
    for name, workspace in [('legacy', baseline), ('calibrated', candidate)]:
        path = workspace/'src/osp_mission/osp_mission/edge_policy.py'
        spec = importlib.util.spec_from_file_location('replay_'+name, path)
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        modules.append((module, path))
    journal = [json.loads(line) for line in (episode/'exploration_memory.jsonl').read_text().splitlines()]
    eligible = []
    for path in sorted((episode/'policy_decisions').glob('*.npz')):
        with np.load(path) as data:
            match = min(journal, key=lambda row: abs(row['elapsed_seconds']-float(data['elapsed'])))
            if abs(match['elapsed_seconds']-float(data['elapsed'])) < .25 and not match['marker_override']:
                eligible.append((path, match))
    if not eligible: raise ValueError('No non-marker-override input with a matching decision journal')
    key = lambda x, y: (round(float(x), 5), round(float(y), 5))
    rows = []
    for index in sorted(set(np.linspace(0, len(eligible)-1, min(6, len(eligible)), dtype=int))):
        path, match = eligible[index]
        with np.load(path) as data:
            retained = {key(row[0], row[1]): row for row in data['candidates']}
            arguments = [data['data'], float(data['resolution']), data['origin'], data['robot'],
                float(data['heading']), data['visits'], data['viewed']]
            arms = []
            for number, (module, _) in enumerate(modules):
                goals, _ = module.options(*arguments, **({'inspection_focal': camera['k'][0]} if number else {}))
                kept = []
                for goal in goals:
                    old = retained.get(key(goal['x'], goal['y']))
                    if old is not None:
                        goal['features'][4] = old[8]; kept.append(goal)
                selected = module.choose(kept, data['weights'])
                arms.append(dict(candidates=len(kept), xy=[selected['x'], selected['y']] if selected else None,
                    features=selected['features'].tolist() if selected else None))
            actual = match['selected']
            rows.append(dict(snapshot=str(path), snapshot_sha256=digest(path), elapsed_sim_s=float(data['elapsed']),
                recorded_selected=actual, legacy_replay=arms[0], calibrated_replay=arms[1],
                legacy_point_matches_recording=bool(arms[0]['xy'] and key(*arms[0]['xy']) == key(*actual[:2])),
                selected_point_changed=arms[0]['xy'] != arms[1]['xy']))
    return dict(evidence_type='offline_retained_endpoint_replay_of_finalized_native_circle_inputs', rows=rows,
        snapshots=len(rows), changed_selected_points=sum(r['selected_point_changed'] for r in rows),
        legacy_replay_agreements=sum(r['legacy_point_matches_recording'] for r in rows),
        empty_new_candidate_sets=sum(r['calibrated_replay']['candidates'] == 0 for r in rows),
        ground_truth_used=False, policy_or_original_result_changed=False, new_trajectory_or_success_simulated=False,
        source_sha256={str(path): digest(path) for _, path in modules}, calibration_source=str(calibration),
        calibration_sha256=digest(calibration), focal_px=camera['k'][0],
        limits=['Retained endpoints and recorded odom-memory cost held fixed;heading-dependent memory is not rerun',
            'Legacy arm includes full-circle gain;candidate combines bounded sector and calibrated readable range',
            'Six evenly indexed non-marker decisions,not a population statistic',
            'Shared960x720camera calibration directly measured in prior native job13',
            'No new frames,recognition,mission duration or success rate simulated'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('episode', 'baseline', 'candidate', 'calibration', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    report = replay(args.episode.resolve(), args.baseline.resolve(), args.candidate.resolve(), args.calibration.resolve())
    save(args.output, report)
    print(json.dumps({k: report[k] for k in ('snapshots', 'changed_selected_points', 'legacy_replay_agreements', 'empty_new_candidate_sets')}))
