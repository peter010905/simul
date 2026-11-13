"""Reproduce a recorded single-frame follow-up before testing faster viewpoints."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np

from continual_native_curriculum import digest,save


def replay(episode,snapshot,workspace,ident):
    path=workspace/'src/osp_mission/osp_mission/marker_focus.py'
    sys.path.insert(0,str(workspace/'src/osp_mission'))
    spec=importlib.util.spec_from_file_location('recorded_single_glance_focus',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    followups=[json.loads(row) for row in (episode/'marker_followups.jsonl').read_text().splitlines()]
    followup=next(row for row in followups if row['id']==ident)
    values=[row for row in json.loads((episode/'marker_observations.json').read_text())
        if row['id']==ident and row['stamp']<=followup['stamp']]
    if len(values)!=1:raise ValueError('This diagnostic requires exactly one recorded image cue')
    data=np.load(snapshot);index=int(data['selected_index'])
    candidates=[dict(x=row[0],y=row[1],yaw=row[2],scan_angle=row[3],distance=row[6],features=row[4:].copy())
        for number,row in enumerate(data['candidates']) if number!=index]
    focus=module.MarkerFocus();focus.observe(values[0],followup['observer'],values[0]['stamp']+.1)
    selected=focus.choose(data['data'],float(data['resolution']),data['origin'],data['robot'],float(data['heading']),
        candidates,{},followup['stamp'])
    goal=[selected[key] for key in ('x','y','yaw')] if selected else None
    matched=bool(selected and np.allclose(goal,data['candidates'][index,:3],atol=1e-9,rtol=0)
        and np.allclose(goal,followup['goal'],atol=1e-9,rtol=0))
    if not matched:raise ValueError('Require exact recorded source choice before drawing a follow-up diagnosis')
    events=[json.loads(row) for row in (episode/'marker_events.jsonl').read_text().splitlines()]
    return dict(evidence_type='exact_recorded_single_glance_followup_replay_not_new_policy_performance',
        episode=str(episode),snapshot=str(snapshot),id=ident,baseline_choice_reproduced=True,
        first_observation_stamp_sim_s=values[0]['stamp'],first_minimum_edge_pixels=values[0]['min_edge_pixels'],
        first_followup=followup,first_followup_range_m=float(np.linalg.norm(np.asarray(goal[:2])-followup['point'])),
        first_followup_was_stationary=bool(selected.get('stationary')),
        recognition_and_approach_journal=[row for row in events if row.get('id')==ident],
        hypothesis='A safe closer viewpoint for a single modest-resolution detection may improve confirmation before leaving;test pixel-aware view costs without weakening ID or approach gates',
        candidate_source_built=False,new_native_speed_improvement_verified=False,ground_truth_policy_inputs=False,
        limits=['A delayed later recognition does not prove why intervening RGB frames did not decode the marker',
            'One captured decision cannot establish total time saved or population reliability',
            'Only the early sensor cue and observed planner map enter the replay;journal timing describes the recorded future'],
        source_sha256={str(p):digest(p) for p in [path,snapshot,episode/'marker_followups.jsonl',episode/'marker_observations.json',episode/'marker_events.jsonl']})


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('episode','snapshot','workspace','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--id',type=int,required=True);args=parser.parse_args()
    result=replay(args.episode.resolve(),args.snapshot.resolve(),args.workspace.resolve(),args.id)
    save(args.output,result);print(json.dumps({k:result[k] for k in ('baseline_choice_reproduced','first_minimum_edge_pixels','first_followup_was_stationary','first_followup_range_m')}))
