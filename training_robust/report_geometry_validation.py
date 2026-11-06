"""Compare the geometry candidate against the same development scenarios."""

import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    root = Path('training_robust/fidelity_validation')
    names = ['endpoint', 'edge', 'symmetric_edge']
    if (root / 'symmetric_learned/completed.json').exists():
        names.append('symmetric_learned')
    runs = {name: json.loads((root / name / 'completed.json').read_text()) for name in names}
    configs = {name: json.loads((root / name / 'configuration.json').read_text()) for name in names}
    assert len({config['scenario_hash'] for config in configs.values()}) == 1
    assert len({json.dumps(config['settings'], sort_keys=True) for config in configs.values()}) == 1
    baseline = {r['seed']: r for r in runs['endpoint']['rows']}
    comparisons = {}
    summaries = {}
    for name, run in runs.items():
        rows = {r['seed']: r for r in run['rows']}
        assert rows.keys() == baseline.keys()
        stats = run['summary']
        summaries[name] = {k: stats[k] for k in ('episodes', 'success_rate', 'mean_seconds', 'successful_mean_seconds', 'collisions', 'mean_found', 'mean_approached')}
        for key in ('by_mount', 'by_obstacles', 'by_frame'):
            summaries[name][key] = {k: v['success_rate'] for k, v in stats[key].items()}
        comparisons[name] = dict(gained=sum(r['success'] and not baseline[s]['success'] for s, r in rows.items()),
                                 lost=sum(not r['success'] and baseline[s]['success'] for s, r in rows.items()),
                                 both_failed=sum(not r['success'] and not baseline[s]['success'] for s, r in rows.items()))
    result = dict(environment='2d_surrogate', split='v2/validation', episodes_per_planner=160,
                  paired_geometries=True, test_loaded=False, gazebo_verified=False,
                  summaries=summaries, compared_with_endpoint=comparisons,
                  note='Development data for policy selection; repeated candidate comparisons are not an independent test')
    (root / 'geometry_comparison.json').write_text(json.dumps(result, indent=2))
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6))
    for axis, key, labels in [(axes[0], 'by_mount', ['walls', 'mixed', 'obstacles', 'hidden_obstacles']),
                              (axes[1], 'by_obstacles', ['8', '12', '16', '24', '32'])]:
        x = np.arange(len(labels)); width = .8 / len(names)
        for i, name in enumerate(names):
            axis.bar(x + (i - (len(names) - 1) / 2) * width,
                     [summaries[name][key][label] * 100 for label in labels], width, label=name)
        axis.set_xticks(x, ['wall', 'mixed', 'obstacle', 'backside'] if key == 'by_mount' else labels)
        axis.set_ylim(0, 105); axis.set_ylabel('3-marker completion (%)')
        axis.set_xlabel('Marker placement' if key == 'by_mount' else 'Requested extra obstacles')
        axis.grid(axis='y', alpha=.2); axis.set_axisbelow(True)
    axes[0].legend(fontsize=8, loc='lower left')
    fig.suptitle('Paired development validation: 160 layouts, 600 s, 5 cm grid, 5 fps\n2D surrogate; no independent-test or Gazebo completion claim', fontsize=11)
    fig.tight_layout(); fig.savefig(root / 'geometry_comparison.png', dpi=160); plt.close(fig)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
