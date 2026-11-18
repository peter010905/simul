"""Score hand-reviewed request frames and pool single-thread warm callback timings."""
import json
from pathlib import Path
import sys
from collections import Counter
import numpy as np

here = Path(__file__).resolve().parent
cohorts = json.loads((here/'cohorts.json').read_text())
review = json.loads((here/'frame_review.json').read_text())
base = Path(sys.argv[1])
labels = {(e['run'],e['t']):e['visual_class'] for e in review['holdout']}
matches = {(e['run'],e['t']) for e in review['reviewed_target']}
out = {}
for group,runs in cohorts.items():
    times, details = [], []
    counts = {policy:Counter() for policy in ('old','new')}
    for run in runs:
        data = json.loads((base/run/'summary.json').read_text())
        times.append(np.load(base/run/'timing.npy'))
        row = dict(run=run,frames=data['frames'],target_reads={k:v for k,v in data['reads'].items() if k in ('7','103')})
        for policy,c in counts.items():
            for e in data['events']:
                event = e[policy]
                if not event:
                    continue
                action = event['action']
                c[action] += 1
                c['dwell_s'] += event['dwell_s']
                if group=='holdout':
                    c[action+'_'+labels[(run,e['t'])]] += 1
                if (run,e['t']) in matches:
                    row.setdefault(policy,dict(t=e['t'],action=action,deg=round(event['yaw_delta_deg'],2),dwell_s=event['dwell_s']))
        details.append(row)
    shared,old,new,decode = np.vstack(times).T
    def stats(a):
        return {k:round(float(v),5) for k,v in zip(('mean','p50','p95'),(a.mean(),*np.percentile(a,[50,95])))}
    out[group]=dict(runs=details,counts=counts,warm_frames=len(shared),ms=dict(
        old_cue=stats(shared+old),new_cue=stats(shared+new),new_dispatch=stats(new),
        cue_delta=stats(new-old),id_decode=stats(decode),old_with_decode=stats(shared+old+decode),
        new_with_decode=stats(shared+new+decode)))
print(json.dumps(out,indent=2))
