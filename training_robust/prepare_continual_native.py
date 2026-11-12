"""Freeze a low-rotation runtime for learning on never-repeated training maps."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np

from continual_native_curriculum import ROOT, HERE, digest, save, source_hashes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    root.mkdir(parents=True, exist_ok=False)
    (root / 'scenarios').mkdir()
    workspace = root / 'runtime'
    subprocess.run([sys.executable, str(ROOT / 'scripts/prepare_low_rotation_runtime.py'),
                    '--destination', str(workspace)], check=True)
    package = workspace / 'src/osp_mission'
    edge = package / 'osp_mission/edge_policy.py'
    code = edge.read_text()
    if code.count('.85*focal*.12*incidence/18') != 1:
        raise ValueError('Review minimum marker-size prior.')
    edge.write_text(code.replace('.85*focal*.12*incidence/18', '.85*focal*.08*incidence/18'))
    # Retain the best previous SLAM/Nav2 settings and initialize CEM at its weights.
    baseline = HERE / 'native90/slam_nav2_search_v1/workspaces/frontier_balanced'
    for name in ('slam.yaml', 'nav2_gazebo.yaml', 'nav2_robot_policy.yaml'):
        shutil.copy2(baseline / 'src/osp_mission/config' / name, package / 'config' / name)
    weights = json.loads((baseline / 'src/osp_mission/config/edge_policy.json').read_text())['parameters']
    shutil.copy2(ROOT / 'src/osp_mission/osp_mission/marker_event_journal.py', package / 'osp_mission')
    evaluator = package / 'osp_mission/gazebo_eval.py'
    code = evaluator.read_text()
    code = code.replace('from osp_mission.runtime_metrics import RuntimeMetrics',
                        'from osp_mission.marker_event_journal import MarkerEventJournal\nfrom osp_mission.runtime_metrics import RuntimeMetrics', 1)
    code = code.replace('status={};detector_status={};metrics=RuntimeMetrics()',
                        "status={};detector_status={};metrics=RuntimeMetrics()\n    journal=MarkerEventJournal(output/'marker_events.jsonl')", 1)
    code = code.replace('status.update(json.loads(message.data))',
                        'status.update(json.loads(message.data))\n        journal.mission(status)', 1)
    old = "    node.create_subscription(String,'mission/detector_status',lambda m:detector_status.update(json.loads(m.data)),10)"
    new = """    def on_detector(message):
        detector_status.update(json.loads(message.data))
        journal.detector(detector_status,status)
    node.create_subscription(String,'mission/detector_status',on_detector,10)"""
    if code.count(old) != 1:
        raise ValueError('Review evaluator marker logging callback.')
    evaluator.write_text(code.replace(old, new))
    config = dict(run_id='nov12-v1', seed=1121200000, workspace=str(workspace),
        baseline_workspace=str(baseline), baseline_weights=weights,
        reserved_manifest=str(HERE / 'native90/reserved_holdout_manifest.json'),
        source_hashes={str(path): source_hashes(path) for path in (workspace, baseline)},
        driver_hashes={str(HERE / name): digest(HERE / name) for name in (
            'continual_native_curriculum.py', 'native_tournament.py', 'gazebo_benchmark.py',
            'run_native_service.py', 'audit_native_campaign.py')},
        population=4, training_episodes_per_generation=20, maximum_native_concurrency=2,
        time_objective='PAR-2, failures cost1200s; require20% lower paired PAR-2',
        validation_gate='>=18/20 success,0 contacts,>=20% lower paired PAR-2',
        final_gate='>=36/40 independently audited native successes,0 contacts',
        train_map_reuse_allowed=False, policy_receives_ground_truth=False,
        known_remaining_issues=['Coverage retention under map changes',
                                'Stale identity conflict recovery', 'Approach status display'])
    save(root / 'configuration.json', config)
    save(root / 'state.json', dict(status='running', generation=0, phase='train', planned=False,
        jobs=[], champion=weights, mean=np.log(weights).tolist(), std=[.4]*5))
    print(json.dumps(dict(root=str(root), workspace=str(workspace), prepared=True)))


if __name__ == '__main__':
    main()
