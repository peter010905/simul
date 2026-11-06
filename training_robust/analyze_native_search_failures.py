"""Offline audit of completed native trials; never supplies truth to a policy.

Counts episodes separately from missed marker IDs. Geometry below is diagnostic:
base bearing is not camera projection, and front-side proximity is not visibility.
"""
import argparse
import collections
import datetime
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml
from scipy.ndimage import distance_transform_edt, label


def read(path):
    return json.loads(path.read_text())


def decisions(episode):
    rows = []
    for path in sorted((episode / 'policy_decisions').glob('*.npz')):
        with np.load(path, allow_pickle=False) as z:
            count = len(z['candidates'])
            row = dict(elapsed=float(z['elapsed']), candidates=count)
            if count == 0:
                data, res = z['data'], float(z['resolution'])
                x, y = ((z['robot'] - z['origin']) / res).astype(int)
                if 0 <= y < data.shape[0] and 0 <= x < data.shape[1]:
                    clearance = distance_transform_edt(data <= 50) * res
                    safe = (data == 0) & (clearance >= .32)
                    safe[y, x] = True
                    groups, _ = label(safe)
                    row.update(robot_cell=int(data[y, x]),
                               clearance_m=float(clearance[y, x]),
                               connected_safe_cells=int(np.count_nonzero(groups == groups[y, x])))
            rows.append(row)
    empty = [r for r in rows if r['candidates'] == 0]
    islands = [r for r in empty if r.get('robot_cell') == 0 and r.get('connected_safe_cells') == 1]
    return dict(total=len(rows), empty=len(empty), singleton_island=len(islands),
                first_island_elapsed=islands[0]['elapsed'] if islands else None,
                last_island_elapsed=islands[-1]['elapsed'] if islands else None,
                first_island=islands[0] if islands else None)


def analyze(root):
    state = read(root / 'state.json')
    rows = []
    hashes = {}
    for job in state['jobs']:
        if job['phase'] != 'validation' or job['status'] != 'done':
            continue
        episode = Path(job['output']) / 'episodes' / str(job['scenario']['seed'])
        path = episode / 'evaluation.json'
        hashes[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        result = read(path)
        truth = result['ground_truth_audit']
        expected, found = set(truth['expected_ids']), set(truth['found_ids'])
        obs = read(episode / 'marker_observations.json')
        counts = collections.Counter(v['id'] for v in obs)
        points = {i: np.median([v['map_point'] for v in obs if v['id'] == i], axis=0)
                  for i in counts}
        scenario = yaml.safe_load(Path(job['scenario']['file']).read_text())
        targets = {t['id']: t for t in scenario['targets']}
        trajectory = np.genfromtxt(episode / 'ground_truth_trajectory.csv', delimiter=',', names=True)
        xy = np.column_stack((trajectory['world_x'], trajectory['world_y']))
        row = dict(number=job['number'], candidate=job['candidate'],
                   obstacles=job['scenario']['actual_obstacles'], mount_profile=job['scenario']['mount_profile'],
                   episode=str(episode), success=result['success'], reason=result['reason'],
                   elapsed=result['elapsed_seconds'], collisions=result['collisions'],
                   expected_ids=sorted(expected), found_ids=sorted(found),
                   near_verified_ids=result['approach']['verified_ids'],
                   missing_ids=sorted(expected-found), extra_ids=sorted(found-expected),
                   correct_positions=truth['correct_positions'],
                   all_actual_approaches=truth['all_approached'],
                   path_length_m=float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()),
                   missing_details=[], extra_details=[], unverified_details=[])
        det = result['detector']
        for ident in row['missing_ids']:
            target = targets[ident]
            dx, dy = xy[:, 0]-target['x'], xy[:, 1]-target['y']
            distance = np.hypot(dx, dy)
            i = int(np.argmin(distance))
            side = (dx*math.cos(target['yaw'])+dy*math.sin(target['yaw']))/np.maximum(distance, 1e-9)
            front = side > .5
            bearing = (np.arctan2(-dy, -dx)-trajectory['world_yaw']+math.pi) % (2*math.pi)-math.pi
            row['missing_details'].append(dict(
                id=ident, size_m=target['size'], closest_m=float(distance[i]),
                closest_sim_stamp=float(trajectory['sim_seconds'][i]),
                closest_base_bearing_degrees=float(np.degrees(bearing[i])),
                closest_front_normal_cosine=float(side[i]),
                closest_front_cone_m=float(distance[front].min()) if front.any() else None,
                decoded_count=det['decoded'].get(str(ident), 0),
                accepted_evidence_count=counts[ident],
                detector_confirmations=det['confirmations'].get(str(ident), 0)))
        for ident in row['extra_ids']:
            distances={str(k):float(np.linalg.norm(points[ident][:2]-points[k][:2]))
                       for k in expected if k in points}
            row['extra_details'].append(dict(id=ident, accepted_count=counts[ident],
                median_map_xyz=points[ident].tolist(), distance_to_real_estimates_m=distances))
        for ident in expected.intersection(found)-set(row['near_verified_ids']):
            evidence = [o for o in obs if o['id']==ident]
            row['unverified_details'].append(dict(id=ident, closest_actual_m=truth['closest_approach_m'][str(ident)],
                accepted_evidence_count=counts[ident],
                minimum_accepted_camera_planar_range_m=min(
                    float(np.linalg.norm(np.array(o['map_point'][:2])-o['camera_origin_map'][:2]))
                    for o in evidence) if evidence else None))
        if result['success']:
            row['category'] = 'success'
        else:
            row['decision_audit'] = decisions(episode)
            log = (episode / 'launch.log').read_text(errors='replace')
            row['spin_commands'] = log.count('for spin behavior.')
            row['progress_failures'] = log.count('Failed to make progress')
            followups = episode / 'marker_followups.jsonl'
            row['followups'] = [json.loads(line) for line in followups.read_text().splitlines()] if followups.exists() else []
            if row['extra_ids']:
                row['category'] = 'false_positive_extra_id'
            elif truth['correct_ids'] and truth['correct_positions'] and truth['all_approached']:
                row['category'] = 'extra_close_rgb_completion_gate'
            elif row['decision_audit']['singleton_island'] >= 20:
                row['category'] = 'persistent_empty_candidate_spin_loop'
            else:
                row['category'] = 'missing_marker_observation_or_confirmation'
        rows.append(row)
    failures = [r for r in rows if not r['success']]
    missed = [m for r in failures for m in r['missing_details']]
    return dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        root=str(root), validation_episodes=len(rows), failed_episodes=len(failures),
        failure_categories=dict(collections.Counter(r['category'] for r in failures)),
        missing_marker_instances=len(missed),
        missing_marker_decoded_zero=sum(m['decoded_count']==0 for m in missed),
        missing_marker_decoded_but_no_accepted=sum(m['decoded_count']>0 and m['accepted_evidence_count']==0 for m in missed),
        missing_marker_partial_accepted=sum(m['accepted_evidence_count']>0 for m in missed),
        missing_marker_actual_near=sum(m['closest_m']<=1.5 for m in missed),
        evaluation_sha256=hashes, episodes=rows,
        limitations=[
            'Decoded counts precede size, border, range, plane and consistency filters; per-ID rejection reasons were not recorded.',
            'Composite RGB video is resized to a small panel. It cannot reproduce native 960-pixel detector thresholds or depth/TF rejection.',
            'Nearest trajectory distance is not evidence of camera visibility. Base bearing and marker-side geometry omit camera offset and occlusion.',
            'The five completion-gate cases remain original timeouts. No counterfactual completion time or improved success rate is asserted.',
            'A safe-grid singleton diagnoses the planner connectivity failure, not physical impossibility of escape. No unsafe clearance reduction is tested here.',
        ])


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    report=analyze(args.root.resolve())
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('episodes','evaluation_sha256','limitations')},indent=2))
