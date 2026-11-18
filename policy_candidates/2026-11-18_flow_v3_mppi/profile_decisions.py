"""Time Flow v3 goal generation, ranking and audit writes on captured robot maps."""
import argparse
import json
from pathlib import Path
import tempfile
import time

import numpy as np
from osp_mission import edge_policy, policy_audit


def profile(folder, destination, focal):
    stages = {'options': [], 'choose': [], 'audit_write': []}
    files = sorted(folder.glob('*.npz'))
    if not files:
        raise ValueError(f'No decision snapshots in {folder}')
    for index, file in enumerate(files):
        with np.load(file) as archive:
            d = {k: archive[k] for k in archive.files}
        args = (d['data'],float(d['resolution']),d['origin'],d['robot'],float(d['heading']),d['visits'],d['viewed'])
        start = time.perf_counter()
        goals, _ = edge_policy.options(*args, inspection_focal=focal, start_ignore_radius=.30)
        ranked = time.perf_counter()
        selected = edge_policy.choose(goals, d['weights'])
        chosen = time.perf_counter()
        policy_audit.save(destination, index, float(d['elapsed']), 'edge', *args, d['weights'], goals, selected)
        saved = time.perf_counter()
        for key, duration in [('options',ranked-start),('choose',chosen-ranked),('audit_write',saved-chosen)]:
            stages[key].append(duration*1000)
    return {'decisions': len(files), 'ms': {key: dict(zip(('p50','p95','max'), np.percentile(values,[50,95,100]).tolist()))
                                            for key, values in stages.items()}}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recordings', nargs='+', type=Path)
    parser.add_argument('--focal', type=float, required=True, help='Recorded camera focal length in pixels')
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as directory:
        for folder in args.recordings:
            print(json.dumps({'folder': str(folder), **profile(folder, directory, args.focal)}), flush=True)
