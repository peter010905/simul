"""Post-run angular agreement of stationary anonymous views; no assumed range for a bearing."""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np

def audit(folder):
    native=folder/'native'
    event_file=native/'marker_nbv.jsonl'
    selected=[r for r in map(json.loads,event_file.read_text().splitlines()) if r.get('nbv_action')=='relook']
    if not selected:
        return []
    scene=json.loads((folder/'scene.json').read_text())
    truth={t['id']:np.array([t['x'],t['y']]) for t in scene['targets']}
    with (folder/'headings.csv').open() as stream:
        poses=np.array([[float(r[k]) for k in ('elapsed_sim_s','x','y','yaw_rad')] for r in csv.DictReader(stream)])
    decisions=[]
    for path in (native/'policy_decisions').glob('*.npz'):
        with np.load(path) as d:
            decisions.append((float(d['elapsed']),d['robot'].copy(),float(d['heading'])))
    rows=[]
    for r in selected:
        t,xy,yaw=min(decisions,key=lambda d:abs(d[0]-r['elapsed']))
        world=min(poses,key=lambda p:abs(p[0]-t))
        if abs(t-r['elapsed'])>.1 or abs(world[0]-t)>.25:
            raise ValueError('Unaligned camera target')
        v=np.array(r['point_map'])-xy
        heading=math.atan2(v[1],v[0])+world[3]-yaw
        errors={}
        for ident,point in truth.items():
            d=point-world[1:3]
            errors[ident]=abs(math.degrees(math.remainder(heading-math.atan2(d[1],d[0]),2*math.pi)))
        best=min(errors,key=errors.get)
        rows.append(dict(t=t,closest_bearing_id=best,angle_error_deg=round(errors[best],2),
                         any_marker_within_12deg=errors[best]<=12.,range_m=r['range_m'],
                         observations=r['observations']))
    return rows

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('folder',type=Path)
    a=p.parse_args()
    print(json.dumps(audit(a.folder),indent=2))
