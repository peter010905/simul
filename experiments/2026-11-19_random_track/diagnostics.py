"""Post-run ground-truth diagnostics. Inputs never feed a navigation policy."""
import csv
import json
import math
from pathlib import Path

import numpy as np


def summarize(folder):
    scene = json.loads((folder / 'scene.json').read_text())
    with (folder / 'headings.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    stamps = np.array([float(row['elapsed_sim_s']) for row in rows])
    xy = np.array([(float(row['x']), float(row['y'])) for row in rows])
    # Sample on integer Gazebo seconds for a reproducible dwell measure.
    times = np.arange(max(0, math.ceil(stamps[0])), math.floor(stamps[-1]) + 1)
    xy = xy[np.searchsorted(stamps, times)]
    width, height = scene['room_size']
    west, east, north, south = -width/2, width/2, height/2, -height/2
    recess = scene['south_recess']
    a, b = recess['x_min'], recess['x_max']
    back = south - recess['depth']
    vertices = [(west,south),(a,south),(a,back),(b,back),(b,south),(east,south),(east,north),(west,north)]
    clearance = np.full(len(times), np.inf)
    for i, start in enumerate(vertices):
        delta = np.asarray(vertices[(i+1) % len(vertices)]) - start
        fraction = np.clip((xy-start) @ delta / np.dot(delta,delta), 0, 1)
        distance = np.linalg.norm(xy - start - fraction[:,None]*delta, axis=1)
        clearance = np.minimum(clearance, distance)
    slow = np.zeros(len(times), dtype=bool)
    for i in range(len(times)-15):
        window = xy[i:i+16]
        if clearance[i:i+16].max() <= .85 and np.linalg.norm(window-window[0],axis=1).max() <= .30:
            slow[i:i+16] = True
    edges = np.flatnonzero(np.diff(np.r_[False,slow,False].astype(int)))
    return dict(near_wall_dwell_intervals_s=[[int(times[i]),int(times[j-1])] for i,j in edges.reshape(-1,2)],
                near_wall_dwell_definition='1 Hz, 15 s windows, center within 0.85 m of perimeter; maximum displacement from window start 0.30 m',
                near_wall_seconds=int(np.count_nonzero(clearance<=.85)),
                box_marker_hosts={t['id']:t['attached_to'] for t in scene['targets'] if not t['attached_to'].startswith('wall_')})


if __name__ == '__main__':
    import sys
    print(json.dumps(summarize(Path(sys.argv[1])), indent=2))
