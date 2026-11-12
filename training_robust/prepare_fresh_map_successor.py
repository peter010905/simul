"""Create a new immutable curriculum from an audited perception candidate."""
import argparse
import copy
import json
from pathlib import Path
import shutil

import yaml

from continual_native_curriculum import HERE, digest, quality, save, source_hashes


TF_ERROR = "Unable to transform robot pose into global plan's frame"


def tf_decision(tf_regression, original):
    path = tf_regression/'regression_state.json'
    if not path.exists():
        return dict(enabled=False, reason='No completed TF comparison;retain audited perception runtime')
    state = json.loads(path.read_text())
    if (state['status'] != 'completed_regression_requires_fresh_map_validation'
            or len(state['jobs']) != 2):
        return dict(enabled=False, reason='TF comparison did not complete')
    baseline = quality([job['baseline'] for job in state['jobs']])
    candidate = quality([job['outcome'] for job in state['jobs']])
    rows = []
    for job in state['jobs']:
        old = next(item for item in original['jobs'] if item['scenario']['seed'] == job['scenario']['seed'])
        before = Path(old['output'])/'episodes'/str(job['scenario']['seed'])
        after = Path(job['output'])/'episodes'/str(job['scenario']['seed'])
        result = json.loads((after/'evaluation.json').read_text())
        audit = result.get('ground_truth_audit', {})
        safe_ids = set(audit.get('found_ids', [])) <= set(audit.get('expected_ids', []))
        safe_positions = max(audit.get('position_errors_m', {}).values(), default=0.) <= .5
        rows.append(dict(seed=job['scenario']['seed'],
            before_tf_errors=(before/'launch.log').read_text().count(TF_ERROR),
            after_tf_errors=(after/'launch.log').read_text().count(TF_ERROR),
            safe_ids=safe_ids, safe_positions=safe_positions))
    enabled = (candidate['contacts'] == 0 and candidate['par2_seconds'] < baseline['par2_seconds']
        and not any(job['outcome']['infrastructure_failure'] for job in state['jobs'])
        and all(row['after_tf_errors'] <= .5*row['before_tf_errors']
                and row['safe_ids'] and row['safe_positions'] for row in rows))
    return dict(enabled=enabled, reason='Require lower paired PAR-2,zero contacts,safe landmarks,and>=50% fewer pose TF errors per case',
                baseline=baseline, candidate=candidate, cases=rows)


def prepare(original_root, perception_workspace, destination, decision):
    original_config = json.loads((original_root/'configuration.json').read_text())
    original_state = json.loads((original_root/'state.json').read_text())
    destination.mkdir(parents=True, exist_ok=False)
    (destination/'scenarios').mkdir()
    workspace = destination/'runtime'
    package = workspace/'src/osp_mission'
    shutil.copytree(perception_workspace/'src/osp_mission', package,
                    ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache'))
    if decision['enabled']:
        path = package/'config/slam.yaml'
        config = yaml.safe_load(path.read_text())
        config['slam_toolbox']['ros__parameters']['restamp_tf'] = True
        path.write_text(yaml.safe_dump(config))
        shutil.copy2(HERE.parent/'scripts/test_tf_restamp_runtime.py', package/'test/test_tf_restamp.py')
    config = copy.deepcopy(original_config)
    config.update(run_id='nov12-v2', seed=1123200000, workspace=str(workspace),
        parent_campaign=str(original_root), perception_runtime=str(perception_workspace),
        tf_selection=decision, purpose='Fresh-map CEM using independently audited small-marker follow-up')
    baseline = Path(config['baseline_workspace'])
    config['source_hashes'] = {str(path): source_hashes(path) for path in (workspace, baseline)}
    config['driver_hashes'] = {name: digest(Path(name)) for name in config['driver_hashes']}
    save(destination/'configuration.json', config)
    save(destination/'state.json', dict(status='running', generation=0, phase='train', planned=False,
        jobs=[], champion=original_state['champion'], mean=original_state['mean'], std=original_state['std'],
        reserved_holdout_consumed=original_state.get('reserved_holdout_consumed', False)))
    return workspace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--original', type=Path, required=True)
    parser.add_argument('--perception-workspace', type=Path, required=True)
    parser.add_argument('--destination', type=Path, required=True)
    parser.add_argument('--tf-regression', type=Path, required=True)
    args = parser.parse_args()
    original = json.loads((args.original/'state.json').read_text())
    decision = tf_decision(args.tf_regression, original)
    print(prepare(args.original.resolve(), args.perception_workspace.resolve(), args.destination.resolve(), decision))


if __name__ == '__main__':
    main()
