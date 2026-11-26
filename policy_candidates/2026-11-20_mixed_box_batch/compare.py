"""Compare paired Gazebo truth traces. Scene geometry is used only after a run."""
import csv
import json
import math
from pathlib import Path
import sys


def read(path):
    return json.loads(path.read_text())


def visits(folder):
    audit = read(folder/'audit.json')
    scene = read(folder/'scene.json')
    boxes = {o['name']: (o['x'], o['y']) for o in scene['obstacles']}
    path = 0.
    previous = current = last = None
    seen = set()
    segments = []
    entered = []
    last_exit = 0.
    exit_s = 0.
    start = None
    minimum = {}
    approached = set()
    markers = {o['id']:(o['x'],o['y']) for o in scene['targets']}
    with (folder/'trajectory.csv').open() as stream:
        for index, row in enumerate(csv.DictReader(stream)):
            if index >= audit['truth_samples']:
                break
            xy = (float(row['world_x']), float(row['world_y']))
            t = float(row['elapsed_sim_s'])
            if start is None:
                start = xy
            if previous is not None:
                path += math.dist(xy, previous)
            previous = xy
            decoded = set(map(int,row['registered_ids'].split()))
            for marker, p in markers.items():
                distance = math.dist(xy, p)
                minimum[marker] = min(minimum.get(marker, math.inf), distance)
                if marker in decoded and distance <= 1.5:
                    approached.add(marker)
            distances = {name: math.dist(xy,p) for name,p in boxes.items()}
            if current is not None:
                if distances[current] <= 2.0:
                    continue
                last_exit, exit_s, current = path, t, None
            nearest = min(distances, key=distances.get)
            if distances[nearest] > 1.75:
                continue
            current = nearest
            if current != last:
                if current in seen and last is not None:
                    segments.append(dict(origin=last,destination=current,exit_s=exit_s,entry_s=t,meters=path-last_exit))
                entered.append(current)
                seen.add(current)
            last = current
    assert approached == {int(i) for i in audit['registered_and_within_s']}
    assert all(abs(minimum[k]-v)<1e-4 for k,v in ((int(k),v) for k,v in audit['min_ground_truth_m'].items()))
    goals = [json.loads(line) for line in (folder/'native/center_wall.jsonl').read_text().splitlines()]
    with (folder/'headings.csv').open() as stream:
        yaw = float(next(csv.DictReader(stream))['yaw_rad'])
    return dict(start_yaw=yaw,run=folder.name,world=audit['world_sha256'],seed=scene['seed'],start=start,
                path_m=path,return_m=sum(s['meters'] for s in segments),returns=segments,
                visits=entered,approached=sorted(approached),registered=audit['final_ids'],
                sim_s=audit['elapsed_sim_s'],bumper=audit['bumper_events'],
                local_batches=sum(g.get('local_batch') is True for g in goals),
                completed_box_fallbacks=sum(g.get('completed_box') is True for g in goals))


def main(base, new):
    result = []
    for entry in read(base/'verified_runs.json'):
        old = base/entry['run']
        revised = new/entry['run'].replace('shared-', 'batch-', 1)
        before, after = visits(old), visits(revised)
        assert read(old/'scene.json') == read(revised/'scene.json')
        assert before['world'] == after['world'] and math.dist(before['start'],after['start']) < .03
        yaw_difference = before['start_yaw'] - after['start_yaw']
        assert abs(math.atan2(math.sin(yaw_difference), math.cos(yaw_difference))) < math.radians(1.)
        result.append(dict(before=before,after=after))
    (new/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    for r in result:
        print([(x['return_m'],x['path_m'],x['approached'],x['sim_s'],x['bumper']) for x in r.values()])


if __name__ == '__main__':
    main(*map(Path,sys.argv[1:]))
