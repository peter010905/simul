"""Offline sensor-evidence replay; never a new Gazebo success or training sample."""
import argparse
import csv
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np


def replay(episode,workspace):
    module_path=workspace/'src/osp_mission/osp_mission/marker_approach.py'
    spec=importlib.util.spec_from_file_location('approach_replay_candidate',module_path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    approach=module.MarkerApproach()
    rows=list(csv.DictReader((episode/'pose_trace.csv').open()))
    odom=np.array([[float(r[k]) for k in ('odom_stamp','odom_x','odom_y')] for r in rows])
    # Repeated/stale odometry rows must not bias interpolation.
    _,indices=np.unique(odom[:,0],return_index=True);odom=odom[np.sort(indices)]
    events=[json.loads(line) for line in (episode/'marker_events.jsonl').read_text().splitlines() if line.strip()]
    observations=json.loads((episode/'marker_observations.json').read_text())
    # Read detector confirmation/retraction timestamps, not evaluator target IDs.
    # The evaluator starts this pose journal only after mission/start succeeds.
    # Its first GT timestamp is used solely as a conservative CLOCK upper bound,
    # never as a robot position. An extra second excludes ambiguous early frames.
    epoch_upper=float(rows[0]['gt_stamp'])+1.
    confirmations={}
    for event in events:
        ident=event.get('id')
        if ident is not None and event.get('event')=='id_confirmed':
            confirmations[int(ident)]=min(confirmations.get(int(ident),float('inf')),
                epoch_upper+event['elapsed_seconds'])
    if not confirmations:
        raise ValueError('Check journal schema: no detector confirmation events')
    approach.reconcile_targets({i:(0,0) for i in confirmations})
    usable=0
    for e in observations:
        stamp=e['stamp']
        if not odom[0,0]<=stamp<=odom[-1,0]:continue
        robot=[float(np.interp(stamp,odom[:,0],odom[:,i])) for i in (1,2)]
        confirmed=stamp>=confirmations.get(int(e['id']),float('inf'))
        approach.observe(e,(0,0),stamp,confirmed=confirmed,odom_robot=robot)
        usable+=1
    sources=[module_path,episode/'pose_trace.csv',episode/'marker_events.jsonl',episode/'marker_observations.json']
    return dict(evidence_type='offline_replay_of_recorded_sensor_inputs',
        new_native_experiment=False,training_sample=False,episode=str(episode),workspace=str(workspace),
        confirmed_ids=sorted(confirmations),observations_replayed=usable,
        range_witness_evidence=approach.range_witnesses,
        odom_pose_source='Interpolation of recorded odometry at the observation timestamp',
        ground_truth_positions_used=False,confirmation_clock_upper_bound=epoch_upper,
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('episode','workspace','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();result=replay(args.episode.resolve(),args.workspace.resolve())
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('source_sha256','range_witness_evidence')},indent=2))
