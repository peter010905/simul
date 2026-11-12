"""Replay only recorded sensor/odom inputs; audit selected proof frames separately."""
import argparse
import copy
import csv
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import yaml

from audit_approach_evidence import physical_at_stamp
from continual_native_curriculum import digest, save


def table(path, columns):
    values = np.asarray([[float(r[k]) for k in columns] for r in csv.DictReader(path.open())])
    values = values[np.argsort(values[:, 0], kind='stable')]
    return values[np.r_[np.diff(values[:, 0]) > 0, True]]


def replay_sensor_inputs(study, workspace):
    # No scenario or independent ground-truth input reaches MarkerApproach.
    sys.path.insert(0, str(workspace/'src/osp_mission'))
    path = workspace/'src/osp_mission/osp_mission/marker_approach.py'
    spec = importlib.util.spec_from_file_location('witness_replay_'+workspace.parent.name, path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    manifest = json.loads((study/'manifest.json').read_text())
    observations = [o for o in json.loads((study/'observations.json').read_text())
        if o['id'] == manifest['replay_id'] and o['stamp'] >= manifest['replay_start_sim_s']]
    odom = table(study/'pose_trace.csv', ('odom_stamp', 'odom_x', 'odom_y'))
    mapped = table(study/'pose_trace.csv', ('map_stamp', 'map_x', 'map_y'))
    approach = module.MarkerApproach(); ident = manifest['replay_id']
    approach.reconcile_targets({ident: observations[0]['map_point'][:2]})
    rows = []; first = None; proof = None
    for observation in observations:
        stamp = observation['stamp']
        if not max(odom[0, 0], mapped[0, 0]) <= stamp <= min(odom[-1, 0], mapped[-1, 0]):
            raise ValueError('Do not extrapolate recorded odom/map poses')
        body = np.array([np.interp(stamp, odom[:, 0], odom[:, k]) for k in (1, 2)])
        position = np.array([np.interp(stamp, mapped[:, 0], mapped[:, k]) for k in (1, 2)])
        distance = float(np.linalg.norm(np.asarray(observation['odom_point'][:2])-body))
        gap = (float(np.linalg.norm(np.asarray(observation['camera_point'])-observation['lidar_camera_point']))
            if observation.get('lidar_camera_point') is not None else None)
        approach.observe(observation, position, stamp+.1, confirmed=True, odom_robot=body)
        if first is None and approach.verified(ident):
            first = stamp; proof = copy.deepcopy(approach.range_witnesses[ident])
        rows.append(dict(stamp_sim_s=stamp, body_relative_distance_m=distance,
            lidar_depth_gap_m=gap, pending_valid_frames=len(approach.witness_runs.get(ident, []))))
    return dict(module=str(path), module_sha256=digest(path), id=ident, rows=rows,
        verified=approach.verified(ident), first_verified_stamp_sim_s=first, first_proof=proof,
        maximum_pending_valid_frames=max(r['pending_valid_frames'] for r in rows),
        ground_truth_inputs=False)


def audit_selected_frames(study, replay):
    # Only this evaluator function reads target positions and independent truth.
    scene = yaml.safe_load((study/'scene.yaml').read_text())
    target = next(t for t in scene['targets'] if t['id'] == replay['id'])
    truth = table(study/'independent_truth_segment.csv', ('sim_seconds', 'world_x', 'world_y'))
    proof = replay['first_proof']
    rows = [physical_at_stamp(truth, (target['x'], target['y']), r['stamp'])
        for r in proof['observations']] if proof else []
    return dict(rows=rows, all_three_actual_approaches_verified=len(rows) == 3 and all(r['verified'] for r in rows),
        actual_task_radius_m=1.5, ground_truth_used_only_for_independent_audit=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('study', 'baseline', 'candidate', 'output'): parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args(); manifest = json.loads((args.study/'manifest.json').read_text())
    if any(digest(Path(p)) != value for p, value in manifest['source_sha256'].items()):
        raise ValueError('Frozen partial sensor study changed')
    old = replay_sensor_inputs(args.study.resolve(), args.baseline.resolve())
    new = replay_sensor_inputs(args.study.resolve(), args.candidate.resolve())
    report = dict(evidence_type='frozen_partial_native_sensor_replay_not_new_native_policy_performance',
        source_episode=manifest['episode'], captured_segment_complete_episode=False,
        study_manifest=str((args.study/'manifest.json').resolve()), baseline=old, candidate=new,
        independent_physical_audit=audit_selected_frames(args.study, new),
        recognition_or_distance_thresholds_changed=False, native_result_modified=False,
        new_native_speed_or_success_verified=False,
        limits=['Robot poses are interpolated from recorded odom/map audit samples',
            'ID848 was already confirmed before replay start190s;replay does not simulate recognition',
            'Selected proof frames are independently checked against actual target/trajectory',
            'No future robot trajectory,time saving or mission success is simulated'])
    save(args.output, report)
    print(json.dumps(dict(legacy_verified=old['verified'], candidate_verified=new['verified'],
        candidate_first_stamp=new['first_verified_stamp_sim_s'],
        physical_proof_verified=report['independent_physical_audit']['all_three_actual_approaches_verified'])))
