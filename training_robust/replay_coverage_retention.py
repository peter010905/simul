"""Compare coverage memory using recorded native RGB-heartbeat/LiDAR/map inputs."""
import argparse
import hashlib
import inspect
import json
from pathlib import Path
import time

import numpy as np

from osp_mission.edge_policy import EdgeCoverage
from osp_mission.persistent_edge_coverage import PersistentEdgeCoverage


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('inputs', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    files = sorted(args.inputs.glob('frame_*.npz'))
    result = {'input_hashes': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
              'note': 'Offline replay diagnoses memory retention, not native mission success.',
              'results': {}}
    for factory in (EdgeCoverage, PersistentEdgeCoverage):
        memory = factory()
        loss = gain = 0
        rows = []
        start = time.monotonic()
        for path in files:
            frame = np.load(path)
            data = frame['data']
            resolution = float(frame['resolution'])
            origin = tuple(frame['origin'])
            before = int(memory.viewed.sum()) if memory.viewed is not None else 0
            memory.grid(data, resolution, origin)
            retained = int(memory.viewed.sum())
            loss += max(0, before - retained)
            memory.observe(data, resolution, origin, frame['camera'], frame['endpoints'],
                           float(frame['stamp']), float(frame['focal']), float(frame['horizontal_fov']))
            gain += max(0, int(memory.viewed.sum()) - retained)
            rows.append(dict(stamp=float(frame['stamp']), viewed=int(memory.viewed.sum()),
                             retained=retained, lost=max(0, before-retained)))
        source = Path(inspect.getfile(factory))
        result['results'][factory.__name__] = dict(
            frames=len(rows), gained_on_frames=gain, lost_on_map_update=loss,
            final_viewed=int(memory.viewed.sum()), anchors=len(getattr(memory, 'anchors', [])),
            wall_seconds=time.monotonic()-start, trace=rows,
            source=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: {a: b for a, b in v.items() if a != 'trace'}
                      for k, v in result['results'].items()}, indent=2))


if __name__ == '__main__':
    main()
