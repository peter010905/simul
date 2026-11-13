"""Independent scoring of a policy-only final SLAM landmark replay.

The replay function receives accepted observations and serialized SLAM poses.
Only the separate evaluator below opens the scenario for coordinate errors.
Original native results are never changed or counted as new native passes.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from candidate_graph_landmarks import finalize,read_graph
from continual_native_curriculum import digest,save


def score(points,scenario,alignment):
    tx,ty,angle=alignment;c,s=math.cos(angle),math.sin(angle);R=np.array([[c,-s],[s,c]])
    return {str(target['id']):float(np.linalg.norm(np.asarray(points[target['id']])-
        (R@np.array([target['x'],target['y']])+[tx,ty]))) for target in scenario['targets']}


def replay(episode,exporter,output):
    output.mkdir(parents=True,exist_ok=False)
    datasets=list(episode.glob('*/map.data'))
    if len(datasets)!=1:raise ValueError('Require one finalized native SLAM dataset')
    data=datasets[0];graph=output/'graph_scan_poses.csv'
    with (output/'export.log').open('w') as stream:
        subprocess.run([str(exporter),str(data),str(graph)],stdout=stream,stderr=stream,check=True,
            env={'LD_LIBRARY_PATH':'/opt/ros/jazzy/lib'},timeout=15)
    markers=data.parent/'markers.csv'
    with markers.open() as stream:original={int(row['id']):(float(row['x']),float(row['y'])) for row in csv.DictReader(stream)}
    observations=episode/'marker_observations.json'
    fixed,report=finalize(json.loads(observations.read_text()),read_graph(graph),original)
    save(output/'policy_reprojection.json',report)
    # Ground truth is opened only after the candidate has produced its result.
    evaluation=json.loads((episode/'evaluation.json').read_text());scenario=Path(evaluation['scenario'])
    cfg=yaml.safe_load(scenario.read_text());alignment=evaluation['ground_truth_audit']['world_to_map']
    before=score(original,cfg,alignment);after=score(fixed,cfg,alignment)
    result=dict(evidence_type='finalized_native_SLAM_graph_replay_not_new_native_success',episode=str(episode),
        original_native_success=evaluation['success'],original_native_seconds=evaluation['elapsed_seconds'],
        original_coordinate_errors_m=before,reprojected_coordinate_errors_m=after,
        all_reprojected_coordinates_within_0_5m=all(v<=.5 for v in after.values()),
        corrected_ids=report['corrected_ids'],policy_inputs_contain_ground_truth=False,
        original_native_results_modified=False,report=report,
        source_sha256={str(p):digest(p) for p in [data,markers,observations,graph,scenario,episode/'evaluation.json',exporter,
            ROOT/'scripts/candidate_graph_landmarks.py']},
        limits=['Final serialized SLAM graph contains later optimization; this is a final map update,not an online localization fix',
            'Native driving time,physical approach credit and collisions are unchanged and are not reclassified by replay',
            'Fixed constants apply across scenes; no target truth or scene geometry enters projection or inlier selection'])
    save(output/'evaluation.json',result);return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode',type=Path);parser.add_argument('--exporter',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    result=replay(args.episode.resolve(),args.exporter.resolve(),args.output.resolve())
    print(json.dumps({k:v for k,v in result.items() if k in ('original_coordinate_errors_m','reprojected_coordinate_errors_m','all_reprojected_coordinates_within_0_5m','corrected_ids')}))
