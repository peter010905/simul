"""Compare pixel-aware follow-up against an exactly reproduced native choice."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
from scipy.ndimage import distance_transform_edt

from continual_native_curriculum import digest,save
from replay_single_glance_focus import replay as verify_baseline


def replay(episode,snapshot,baseline,candidate,ident):
    original=verify_baseline(episode,snapshot,baseline,ident)
    path=candidate/'src/osp_mission/osp_mission/marker_focus.py'
    sys.path.insert(0,str(candidate/'src/osp_mission'))
    spec=importlib.util.spec_from_file_location('pixel_focus_shadow',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    from osp_mission.edge_policy import visible
    followup=original['first_followup'];data=np.load(snapshot);index=int(data['selected_index'])
    evidence=next(row for row in json.loads((episode/'marker_observations.json').read_text())
        if row['id']==ident and row['stamp']==original['first_observation_stamp_sim_s'])
    choices=[dict(x=row[0],y=row[1],yaw=row[2],scan_angle=row[3],distance=row[6],features=row[4:].copy())
        for number,row in enumerate(data['candidates']) if number!=index]
    focus=module.MarkerFocus();focus.observe(evidence,followup['observer'],evidence['stamp']+.1)
    selected=focus.choose(data['data'],float(data['resolution']),data['origin'],data['robot'],float(data['heading']),
        choices,{},followup['stamp'])
    if selected is None:raise ValueError('Pixel-aware replay returned no follow-up')
    point=np.array([selected['x'],selected['y']]);target=np.array(followup['point'])
    grid=data['data'];resolution=float(data['resolution']);origin=data['origin']
    clearance=distance_transform_edt(grid<=50)*resolution
    safe=np.where((grid==0)&(clearance>=.32),0,100)
    start=(data['robot']-origin)/resolution;stop=(point-origin)/resolution
    route=bool(visible(safe,start[1],start[0],np.array([stop[1]]),np.array([stop[0]]))[0])
    radius=float(np.linalg.norm(point-target));endpoint=(target+.25*(point-target)/radius-origin)/resolution
    sightline=bool(visible(grid,stop[1],stop[0],np.array([endpoint[1]]),np.array([endpoint[0]]))[0])
    selected={k:(v.tolist() if isinstance(v,np.ndarray) else v.item() if isinstance(v,np.generic) else v)
        for k,v in selected.items()}
    return dict(evidence_type='same_observed_inputs_pixel_followup_shadow_not_new_native_success',
        baseline=original,candidate_workspace=str(candidate),candidate_source_sha256=digest(path),
        candidate=selected,candidate_range_to_marker_m=radius,
        known_free_inflated_direct_route=route,known_free_marker_sightline=sightline,
        source_inputs_contain_ground_truth=False,new_native_speed_or_success_verified=False,
        limits=['Image-size scaling is a heuristic estimate;actual decoding must be tested in Gazebo',
            'One early decision does not simulate a new path or establish mission time saved',
            'Original native result and journal are preserved;ID and actual approach thresholds are unchanged'])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('episode','snapshot','baseline','candidate','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--id',type=int,required=True);args=parser.parse_args()
    result=replay(*(getattr(args,k).resolve() for k in ('episode','snapshot','baseline','candidate')),args.id)
    save(args.output,result);print(json.dumps({k:result[k] for k in ('candidate_range_to_marker_m','candidate','known_free_inflated_direct_route','known_free_marker_sightline')}))
