"""Post-hoc coordinate diagnosis. Ground truth never enters the detector."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    rows = []
    for run in args.runs:
        evaluation = json.loads((run/'evaluation.json').read_text())
        observations = json.loads((run/'marker_observations.json').read_text())
        scenario = yaml.safe_load(Path(evaluation['scenario']).read_text())
        audit = evaluation['ground_truth_audit']
        tx, ty, yaw = audit['world_to_map']
        c, s = math.cos(yaw), math.sin(yaw)
        for target in scenario['targets']:
            points = [row['map_point'][:2] for row in observations
                      if row['id'] == target['id'] and row.get('map_point') is not None]
            if not points:
                continue
            expected = np.array([tx+c*target['x']-s*target['y'], ty+s*target['x']+c*target['y']])
            estimate = np.median(points[-30:], axis=0)
            rows.append(dict(run=str(run), id=target['id'], observations=len(points),
                             recorded_final_error_m=audit['position_errors_m'].get(str(target['id'])),
                             retrospective_map_anchor_error_m=float(np.linalg.norm(estimate-expected))))
    report = dict(rows=rows, note='Accepted observations replay only; does not change any native verdict or prove counterfactual mission success.')
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
