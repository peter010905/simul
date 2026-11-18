"""Save the planner's observed inputs for retrospective native-run diagnosis."""
from pathlib import Path

import numpy as np


def save(directory, index, elapsed, planner, data, resolution, origin,
         robot, heading, visits, viewed, weights, candidates, selected):
    root = Path(directory).expanduser() / 'policy_decisions'
    root.mkdir(parents=True, exist_ok=True)
    columns = ['x', 'y', 'yaw', 'scan_angle', 'surface_gain', 'frontier_gain',
               'distance', 'turn', 'revisit']
    rows = [[c['x'], c['y'], c['yaw'], c.get('scan_angle', 0.), *c['features']]
            for c in candidates]
    selected_index = next((i for i, c in enumerate(candidates) if c is selected), -1)
    path = root / f'{index:04d}.npz'
    temporary = path.with_suffix('.tmp')
    with temporary.open('wb') as stream:
        np.savez_compressed(stream, elapsed=elapsed, planner=planner, data=data,
                            resolution=resolution, origin=origin, robot=robot,
                            heading=heading, visits=np.asarray(visits).reshape(-1, 2),
                            viewed=viewed if viewed is not None else np.zeros(0, dtype=bool),
                            weights=weights, candidate_columns=columns,
                            candidates=np.asarray(rows).reshape(-1, len(columns)),
                            selected_index=selected_index)
    temporary.replace(path)
