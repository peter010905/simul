"""Cross-check measured ranges against the saved path and summarize raw R32 observations."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics


def summarize(trial_dir, targets):
    data = json.loads((trial_dir/'metrics.json').read_text())
    start = next(event for event in map(json.loads, (trial_dir/'events.jsonl').read_text().splitlines())
                 if event['event'] == 'start_ack')
    min_distance = {k: math.inf for k in targets}
    entered = {}
    witnessed = {}
    pair_time = None
    bump = False
    edges_on_trace = 0
    frames = 0
    shutdown_samples = 0
    simultaneous = False
    with (trial_dir/'trajectory.csv').open() as source:
        for row in csv.DictReader(source):
            # The stop RPC spins the node briefly after the time of verdict.
            if frames == data['truth_samples']:
                shutdown_samples += 1
                continue
            frames += 1
            now = float(row['elapsed_sim_s'])
            ids = {int(i) for i in row['registered_ids'].split()}
            simultaneous |= targets.keys() <= ids
            position = (float(row['world_x']), float(row['world_y']))
            contact = row['bumper'] == 'True'
            edges_on_trace += int(contact and not bump)
            bump = contact
            for ident, target in targets.items():
                distance = math.dist(position, target)
                min_distance[ident] = min(min_distance[ident], distance)
                if distance <= 1.5:
                    entered.setdefault(ident,now)
                    if ident in ids:
                        witnessed.setdefault(ident,now)
            if pair_time is None and targets.keys() <= ids and targets.keys() <= witnessed.keys():
                pair_time = now
    assert frames == data['truth_samples'] and data['hazard_packets'] > 0
    for ident, minimum in min_distance.items():
        assert abs(data['min_ground_truth_m'][str(ident)]-minimum)<1e-6
    for audit, label in [(entered,'first_within_s'), (witnessed,'registered_and_within_s')]:
        assert set(audit)==set(map(int,data[label]))
        assert all(abs(v-data[label][str(i)])<=.0001 for i,v in audit.items())
    assert (pair_time is None) == (data['pair_time_s'] is None)
    assert pair_time is None or abs(pair_time-data['pair_time_s'])<=.0001
    result = {k:v for k,v in data.items() if k!='last_policy_status'}
    result.update(post_verdict_samples_excluded=shutdown_samples, trial=trial_dir.name, simultaneous_registration=simultaneous, start_world_xy=start['pose'],
                  truth_bumper_edges=edges_on_trace, trajectory_sha256=hashlib.sha256((trial_dir/'trajectory.csv').read_bytes()).hexdigest())
    if data['policy']=='r32':
        manifest=next((trial_dir/'native').glob('*/latest.json'))
        last=json.loads(manifest.read_text())
        marker_file=manifest.parent/last['snapshot']/'markers.csv'
        last_status=data['last_policy_status']
        result.update(policy_elapsed_s=last_status.get('elapsed'), remaining_s=last_status.get('remaining'),
                      planning=last_status.get('planning',{}))
        raw=manifest.parent/'observations.jsonl'
        bearings=0
        gaps=[]
        for row in map(json.loads,raw.read_text().splitlines()):
            if not start['sim'] <= row['stamp'] <= start['sim']+data['elapsed_sim_s']:
                continue
            bearings += int(targets.keys() <= {x['id'] for x in row['bearings']})
            positions={x['id']: x['point'][:2] for x in row['detections']}
            if targets.keys() <= positions.keys():
                gaps.append(math.dist(positions[7],positions[103]))
        result.update(frames_with_both_decoded_ids=bearings, frames_with_both_metric_ids=len(gaps),
                      median_pair_separation_m=statistics.median(gaps) if gaps else None)
    else:
        marker_file=next((trial_dir/'native').glob('*/markers.csv'))
    with marker_file.open() as source:
        coords={int(r['id']):[float(r['x']),float(r['y'])] for r in csv.DictReader(source)}
    result.update(saved_ids=sorted(coords),saved_map_xy=coords)
    assert result['saved_ids']==result['final_ids']
    return result


if __name__=='__main__':
    import yaml
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path)
    args=parser.parse_args()
    scene=yaml.safe_load((args.root/'scene.yaml').read_text())
    targets={t['id']:(t['x'],t['y']) for t in scene['targets']}
    assert math.isclose(math.dist(*targets.values()),.1,abs_tol=1e-8)
    results=[summarize(p.parent,targets) for p in sorted(args.root.glob('*/metrics.json'))]
    assert len({r['world_sha256'] for r in results})==1
    print(json.dumps(results,indent=2))
