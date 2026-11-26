"""Replay the recorded 20aecbcabc F1/F2 fit and emit requests for the offline NavFn audit."""
import argparse
import json
import math
import subprocess
from pathlib import Path

import numpy as np

from osp_mission._r32_navigation_core import Grid
from osp_mission.adaptive_faces import clearance_grid, indices
from osp_mission.box_views import local_boxes
from osp_mission.center_wall import CenterWall


def replay(data, out):
    manifest = data/'manifest.json'
    if not manifest.exists():
        manifest = data/'metadata.json'
    records = json.loads(manifest.read_text())['snapshots']
    out.mkdir(parents=True, exist_ok=True)
    selected = []
    queries = []
    for snapshot in records:
        key = snapshot['key']
        if key not in ('0005', '0006'):
            continue
        pose = snapshot['pose']
        with np.load(data/(key+'.npz')) as z:
            grid = Grid(z['slam'], snapshot['slam'][2], tuple(snapshot['slam'][3:]))
            cost = Grid(z['cost'], snapshot['cost'][2], tuple(snapshot['cost'][3:]))
        fitted = local_boxes(grid, pose)
        recorded = snapshot['boxes'][0]['center']
        box, faces = min(fitted, key=lambda item: np.linalg.norm(item[0]-recorded))
        if np.linalg.norm(box-recorded) > .01:
            raise ValueError('The recorded box fit was not reproduced')
        # Isolate each face, because the runtime dispatches only one at a time.
        # Actual RGB coverage is not asserted from these geometry snapshots.
        viewed = np.zeros((4,)+grid.cells.shape, bool)
        for side in (0, 1):
            seen = [math.atan2(n[1],n[0]) for i,(_,n) in enumerate(faces) if i != side]
            policy = CenterWall()
            policy.phase = 'wall'
            goal = policy.decide(grid, cost, pose, (0.,0.,0.), viewed, snapshot['elapsed'],
                                 600., lambda _:False, fitted=[(box,faces,seen)])
            if not goal or goal.get('fitted_face') != side+1 or policy.phase != 'interior':
                raise ValueError(f'Missing expected face F{side+1} at {key}')
            label = f'{key}-F{side+1}'
            selected.append(dict(label=label, **{k:goal[k] for k in
                                 ('x','y','yaw','standoff_m','lateral_m','clearance_m')}))
            w,h,res,ox,oy,rotation = snapshot['raw']
            if abs(rotation) > 1e-6:
                raise ValueError('The offline NavFn audit expects an unrotated costmap')
            query = [label, data.resolve()/(key+'.raw'), w,h,res,ox,oy,*pose[:2],goal['x'],goal['y'],goal['yaw']]
            queries.append(' '.join(map(str,query)))
    if len(selected) != 4:
        raise ValueError('Both decision snapshots are required')
    (out/'selected.json').write_text(json.dumps(selected,indent=2)+'\n')
    (out/'queries').write_text('\n'.join(queries)+'\n')


def verify_navfn(data, out, binary):
    """Check the returned path, including NavFn tolerance and unknown cells."""
    subprocess.run([str(binary), str(out/'queries')], check=True)
    records_file = data/'manifest.json' if (data/'manifest.json').exists() else data/'metadata.json'
    snapshots = {r['key']:r for r in json.loads(records_file.read_text())['snapshots']}
    goals = {r['label']:r for r in json.loads((out/'selected.json').read_text())}
    checked = {}
    for line in (out/'queries.paths').read_text().splitlines():
        key, *items = line.split()
        if not items or items[0]=='ERR':
            raise ValueError(f'NavFn did not produce a path for {key}')
        tag = key.split('-')[0]
        record, goal = snapshots[tag], goals[key]
        path = np.array([record['pose'][:2], *[list(map(float, p.split(','))) for p in items]])
        distance = np.linalg.norm(np.diff(path,axis=0),axis=1)
        fine = np.concatenate([np.linspace(a,b,max(2,math.ceil(length/.02)+1))
                               for a,b,length in zip(path,path[1:],distance)])
        with np.load(data/(tag+'.npz')) as z:
            slam = Grid(z['slam'], record['slam'][2], tuple(record['slam'][3:]))
            raw = Grid(z['raw'], record['raw'][2], tuple(record['raw'][3:]))
        for grid in (raw, slam):
            rows, cols = indices(grid, fine)
            if ((rows < 0) | (cols < 0) | (rows >= grid.cells.shape[0]) | (cols >= grid.cells.shape[1])).any():
                raise ValueError(f'Path outside recorded map for {key}')
        error = float(np.linalg.norm(path[-1]-[goal['x'],goal['y']]))
        max_cost = int(raw.cells[indices(raw, fine)].max())
        minimum = float(clearance_grid(slam)[indices(slam, fine)].min())
        if error > .005 or max_cost >= 253 or minimum < .32:
            raise ValueError(f'{key} path failed: error={error}, raw_cost={max_cost}, clearance={minimum}')
        checked[key] = dict(goal, path_m=float(distance.sum()), endpoint_error_m=error,
                            max_raw_cost=max_cost, min_path_slam_clearance_m=minimum)
    if set(checked)!=set(goals):
        raise ValueError('Missing returned plans')
    (out/'verified.json').write_text(json.dumps(checked,indent=2)+'\n')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('data', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--navfn', type=Path, help='Offline executable from the viewpoint audit')
    args = p.parse_args()
    replay(args.data, args.output)
    if args.navfn:
        verify_navfn(args.data, args.output, args.navfn)
