"""Align robot trajectory poses to evaluate saved marker locations after a trial."""
import csv
import json
from pathlib import Path

import numpy as np


def measure(folder):
    audit = json.loads((folder/'audit.json').read_text())
    scene = json.loads((folder/'scene.json').read_text())
    with (folder/'headings.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    seconds = np.array([float(row['elapsed_sim_s']) for row in rows])
    truth = np.array([[float(row['x']),float(row['y'])] for row in rows])
    pairs = []
    for file in sorted((folder/'native/policy_decisions').glob('*.npz')):
        with np.load(file,allow_pickle=False) as decision:
            t=float(decision['elapsed'])
            if seconds[0]<=t<=min(audit['elapsed_sim_s'],seconds[-1]):
                xy=[np.interp(t,seconds,truth[:,axis]) for axis in (0,1)]
                pairs.append((decision['robot'][:2],xy))
    if len(pairs)<3:
        raise ValueError('Insufficient timestamped pose pairs')
    local,world=np.transpose(np.array(pairs),(1,0,2))
    if np.linalg.matrix_rank(local-local.mean(axis=0))<2:
        raise ValueError('Trajectory does not span the plane')
    u,_,vt=np.linalg.svd((local-local.mean(axis=0)).T@(world-world.mean(axis=0)))
    rot=u @ np.diag([1.,np.linalg.det(u@vt)]) @ vt
    shift=world.mean(axis=0)-local.mean(axis=0)@rot
    residual=np.linalg.norm(local@rot+shift-world,axis=1)
    targets={str(t['id']):[t['x'],t['y']] for t in scene['targets']}
    errors={k:round(float(np.linalg.norm(np.asarray(xy)@rot+shift-targets[k])),3)
            for k,xy in audit['saved_map_xy'].items() if k in targets}
    return dict(run=folder.name,matched_poses=len(pairs),
                pose_alignment_rms_m=round(float(np.sqrt(np.mean(residual**2))),3),
                marker_position_errors_m=errors)


if __name__=='__main__':
    import sys
    print(json.dumps([measure(Path(name)) for name in sys.argv[1:]],indent=2))
