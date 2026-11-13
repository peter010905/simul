"""Additional diagnostic shadows; never consume qualification or holdout runs."""
import argparse
import json
from pathlib import Path

import numpy as np

from continual_native_curriculum import digest,save
from replay_pixel_focus import replay


def batch(roots,baseline,candidate):
    expected=digest(baseline/'src/osp_mission/osp_mission/marker_focus.py')
    seen=set();results=[];skipped=[]
    for root in roots:
        for job in json.loads((root/'state.json').read_text())['jobs']:
            if job['status']!='done' or job.get('phase') in ('holdout','spatial_holdout','validation','pilot'):continue
            episode=Path(job['output'])/'episodes'/str(job['scenario']['seed'])
            if str(episode) in seen:continue
            seen.add(str(episode));source=Path(job['workspace'])/'src/osp_mission/osp_mission/marker_focus.py'
            if digest(source)!=expected:continue
            followups=episode/'marker_followups.jsonl';observations=episode/'marker_observations.json'
            if not followups.exists() or not observations.exists():continue
            hints=json.loads(observations.read_text());first={}
            for line in followups.read_text().splitlines():
                row=json.loads(line);first.setdefault(row['id'],row)
            snapshots=[]
            for file in sorted((episode/'policy_decisions').glob('*.npz')):
                with np.load(file) as data:
                    index=int(data['selected_index'])
                    if index>=0:snapshots.append((file,data['candidates'][index,:3].copy()))
            for ident,row in first.items():
                early=[h for h in hints if h['id']==ident and h['stamp']<=row['stamp']]
                if len(early)!=1:continue
                matching=[file for file,goal in snapshots if np.allclose(goal,row['goal'],atol=1e-9,rtol=0)]
                if len(matching)!=1:
                    skipped.append(dict(episode=str(episode),id=ident,reason='followup_not_uniquely_selected_in_final_planner_audit'));continue
                try:
                    result=replay(episode,matching[0],baseline,candidate,ident)
                    result['shape']=job['scenario'].get('arena',{}).get('shape','unspecified')
                    results.append(result)
                except ValueError as exc:skipped.append(dict(episode=str(episode),id=ident,reason=str(exc)))
    return dict(evidence_type='additional_recorded_single_glance_shadows_not_native_generalization',
        cases=len(results),shapes=sorted({r['shape'] for r in results}),results=results,skipped=skipped,
        qualification_or_holdout_data_consumed=False,ground_truth_policy_inputs=False,
        limits=['Only uniquely matched first single-frame cues with unchanged baseline focus code are included',
            'Other pending ID hints are absent;candidate choice is conditional on the recorded selected ID',
            'Straight-line blockage is not necessarily an unsafe global Nav2 route for ordinary planner candidates',
            'No native mission success,time saving,or transfer rate can be inferred from these shadows'])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,action='append',required=True)
    for key in ('baseline','candidate','output'):parser.add_argument('--'+key,type=Path,required=True)
    args=parser.parse_args();result=batch([p.resolve() for p in args.root],args.baseline.resolve(),args.candidate.resolve())
    save(args.output,result);print(json.dumps(dict(cases=result['cases'],shapes=result['shapes'],skipped=len(result['skipped']))))
