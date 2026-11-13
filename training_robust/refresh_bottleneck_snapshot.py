"""Refresh descriptive training statistics from completed authoritative episodes."""
import argparse
from collections import Counter
import datetime
import hashlib
import json
from pathlib import Path

import numpy as np


def refresh(campaign,output):
    report=json.loads(output.read_text());state=json.loads((campaign/'state.json').read_text());rows=[]
    for job in state['jobs']:
        if job['status']!='done':continue
        episode=Path(job['output'])/'episodes'/str(job['scenario']['seed'])
        path=episode/'evaluation.json';evaluation=json.loads(path.read_text())
        analysis=json.loads((Path(job['output'])/'observer_analysis.json').read_text())
        audit=evaluation['ground_truth_audit'];outcome=job['outcome']
        category=('none' if outcome['success'] else 'input_verification_missing' if outcome['infrastructure_failure']
                  else 'marker_recognition' if not audit['correct_ids'] else 'map_localization' if not audit['correct_positions'] else 'other')
        rows.append(dict(number=job['number'],role=job['role'],shape=job['scenario']['arena']['shape'],
            seed=job['scenario']['seed'],parameters=job['parameters'],outcome=outcome,failure_category=category,
            exact_ids=audit['correct_ids'],map_positions=audit['correct_positions'],
            geometric_closest_passes_all3=audit['all_approached'],
            sensor_verified_ids=evaluation.get('approach',{}).get('verified_ids',[]),
            coordinate_errors_m=audit['position_errors_m'],wall_s=evaluation['wall_seconds'],
            rtf=evaluation['runtime_metrics']['real_time_factor'],motion=analysis['motion'],revisit=analysis['revisits'],
            authoritative_evaluation=str(path),evaluation_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    if not rows:raise ValueError('No completed authoritative training episodes')
    def rate(entries):
        return dict(successes=sum(e['outcome']['success'] for e in entries),completed=len(entries),
                    percent=100*sum(e['outcome']['success'] for e in entries)/len(entries))
    verified=[r for r in rows if not r['outcome']['infrastructure_failure']]
    report.update(captured_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        completed_native_rows=rows,mixed_coefficient_training=rate(rows),environment_verified_training=rate(verified),
        fixed_incumbent=rate([r for r in rows if r['role']=='0']),
        infrastructure_unverified_completed=len(rows)-len(verified),
        zero_collision_completed=sum(r['outcome']['contacts']==0 for r in rows),
        correct_all_three_ID_missions=sum(r['exact_ids'] for r in rows),
        geometric_all_three_closest_pass_missions=sum(r['geometric_closest_passes_all3'] for r in rows),
        failures=dict(Counter(r['failure_category'] for r in rows if not r['outcome']['success'])))
    for key in ('observed_sim_s','stationary_sim_s','stationary_turn_sim_s'):
        report['motion'][key]=sum(r['motion'][key] for r in rows)
    m=report['motion'];m['stationary_plus_turn_fraction']=(m['stationary_sim_s']+m['stationary_turn_sim_s'])/m['observed_sim_s']
    rtf=[r['rtf'] for r in rows];times=[r['outcome']['seconds'] for r in rows if r['outcome']['success']]
    report['rtf']=dict(minimum=min(rtf),maximum=max(rtf),median=float(np.median(rtf)))
    report['successful_mission_time_sim_s']=dict(mean=float(np.mean(times)),median=float(np.median(times)))
    output.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    return report['mixed_coefficient_training']


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--campaign',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();print(refresh(args.campaign.resolve(),args.output.resolve()))
