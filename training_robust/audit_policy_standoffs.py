"""Evaluator-only physical feasibility of the policy's conservative standoffs.

This diagnostic never removes a scene or supplies a target to the policy.
Actual task radius remains1.5m; native observed-map feasibility is separate.
"""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml

from continual_native_curriculum import digest, save


def audit(campaign, workspace):
    sys.path.insert(0, str(workspace/'src/osp_mission'))
    from osp_mission.space_geometry import accessible_space, rasterize_space
    from osp_mission.edge_policy import visible
    state = json.loads((campaign/'state.json').read_text()); rows = []; sources = {}
    for job in state['jobs']:
        scene = Path(job['scenario']['file']); sources[str(scene)] = digest(scene)
        cfg = yaml.safe_load(scene.read_text()); resolution = .05
        safe = accessible_space(cfg, resolution); data = rasterize_space(cfg, resolution).astype(int)*100
        for target in cfg['targets']:
            reachable = []; point = np.array([target['x'], target['y']])
            for distance in np.round(np.arange(.65, 1.501, .05), 2):
                for offset in np.linspace(-.45, .45, 13):
                    angle = target['yaw']+offset
                    ray = np.array([math.cos(angle), math.sin(angle)])
                    position, end = point+distance*ray, point+.25*ray
                    x, y = np.floor(position/resolution).astype(int)
                    if (0 <= y < safe.shape[0] and 0 <= x < safe.shape[1] and safe[y, x]
                            and visible(data, position[1]/resolution, position[0]/resolution,
                                np.array([end[1]/resolution]), np.array([end[0]/resolution]))[0]):
                        reachable.append(float(distance)); break
            rows.append(dict(shape=cfg['arena']['shape'], seed=cfg['seed'], id=target['id'],
                safe_visible_standoff_distances_m=reachable,
                has_policy_tight_standoff=any(d in (.75, .85, .9) for d in reachable),
                has_body_pose_inside1_3m=any(d <= 1.3 for d in reachable),
                has_task_pose_inside1_5m=bool(reachable)))
    return dict(evidence_type='offline_evaluator_geometry_only_policy_standoff_feasibility_diagnostic', rows=rows,
        total_targets=len(rows), tight_goal_targets=sum(r['has_policy_tight_standoff'] for r in rows),
        inner_radius_targets=sum(r['has_body_pose_inside1_3m'] for r in rows),
        task_radius_targets=sum(r['has_task_pose_inside1_5m'] for r in rows),
        assumptions='5cmraster;0.34mclearance;13front-facing rays+0.25mendpointoffset;globalreachability;notNav2/SLAM',
        ground_truth_available_to_policy=False, scenarios_or_policy_modified=False, source_sha256=sources,
        limits=['Physical offline feasibility is not observed-map or native navigation feasibility',
            'No queued map is removed or changed based on this diagnostic',
            'Body1.3m and policy0.9mstandoffs are internal choices;actualtask remains1.5m'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('campaign', 'workspace', 'output'): parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args(); report = audit(args.campaign.resolve(), args.workspace.resolve())
    save(args.output, report)
    print(json.dumps({k: report[k] for k in ('total_targets', 'tight_goal_targets', 'inner_radius_targets', 'task_radius_targets')}))
