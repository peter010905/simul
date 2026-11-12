"""Observed-input replay of camera-range reward;not a simulated trajectory or success result."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

from continual_native_curriculum import save,source_hashes


def replay(campaign,workspace,calibration):
    sys.path.insert(0,str(workspace/'src/osp_mission'))
    from osp_mission.edge_policy import options,choose
    camera=json.loads(calibration.read_text())
    if not camera['native_identity_verified_after_observation']:raise ValueError('Require bound native CameraInfo')
    focal=camera['k'][0];state=json.loads((campaign/'state.json').read_text());rows=[]
    for job in state['jobs']:
        if job['status']!='done' or job['role']!='0':continue
        episode=Path(job['output'])/'episodes'/str(job['scenario']['seed'])
        journal=[json.loads(line) for line in (episode/'exploration_memory.jsonl').read_text().splitlines()]
        eligible=[]
        for path in sorted((episode/'policy_decisions').glob('*.npz')):
            with np.load(path) as data:
                elapsed=float(data['elapsed'])
                match=min(journal,key=lambda r:abs(r['elapsed_seconds']-elapsed)) if journal else None
                if match and abs(match['elapsed_seconds']-elapsed)<.25 and not match['marker_override']:
                    eligible.append(path)
        if not eligible:continue
        for path in sorted(set([eligible[len(eligible)//2],eligible[-1]])):
            with np.load(path) as d:
                key=lambda x,y:(round(float(x),5),round(float(y),5))
                retained={key(row[0],row[1]):row for row in d['candidates']}
                arguments=[d['data'],float(d['resolution']),d['origin'],d['robot'],float(d['heading']),d['visits'],d['viewed']]
                arms=[]
                for actual_focal in [None,focal]:
                    goals,_=options(*arguments,inspection_focal=actual_focal)
                    kept=[]
                    for goal in goals:
                        old=retained.get(key(goal['x'],goal['y']))
                        if old is None:continue
                        # Preserve recorded odometry memory cost and retained endpoints in both arms.
                        goal['features'][4]=old[8];kept.append(goal)
                    selected=choose(kept,d['weights']);arms.append((kept,selected))
                old_goals,old=arms[0];new_goals,new=arms[1]
                same=next((g for g in new_goals if old is not None and key(g['x'],g['y'])==key(old['x'],old['y'])),None)
                rows.append(dict(shape=job['scenario']['arena']['shape'],snapshot=str(path),elapsed_sim_s=float(d['elapsed']),
                    legacy_retained_candidates=len(old_goals),calibrated_retained_candidates=len(new_goals),
                    legacy_selected_xy=[old['x'],old['y']] if old else None,
                    legacy_selected_surface_gain_m=float(old['features'][0]) if old else None,
                    same_point_calibrated_surface_gain_m=float(same['features'][0]) if same else 0.,
                    calibrated_selected_xy=[new['x'],new['y']] if new else None,
                    calibrated_selected_surface_gain_m=float(new['features'][0]) if new else None,
                    selected_point_changed=bool(old and new and key(old['x'],old['y'])!=key(new['x'],new['y']))))
    return dict(evidence_type='offline_replay_of_saved_native_policy_inputs',rows=rows,
        native_camera_calibration=camera,source_hashes=source_hashes(workspace),
        loaded_policy_module=str(Path(sys.modules[options.__module__].__file__).resolve()),
        snapshots=len(rows),families=sorted({row['shape'] for row in rows}),
        changed_selected_points=sum(row['selected_point_changed'] for row in rows),
        empty_calibrated_candidate_sets=sum(row['calibrated_retained_candidates']==0 for row in rows),
        native_speed_or_success_improvement_verified=False,policy_or_training_results_modified=False,
        ground_truth_used=False,limits=[
            'Both arms use bounded sweeps;comparison isolates range reward against the legacy bare-options API',
            'Old retained endpoints and odometry revisit penalties are held fixed;new heading-dependent memory filtering is not replayed',
            'No new sensor frames,robot trajectory,marker recognition or completion time are simulated',
            'Up to two selected non-marker-override inputs per available family;not a population performance estimate'])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['campaign','workspace','calibration','output']:parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();save(args.output,replay(args.campaign.resolve(),args.workspace.resolve(),args.calibration.resolve()))
