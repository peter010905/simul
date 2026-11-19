"""Attribute each dispatched box inspection to its independently recorded simulator host."""
import bisect
from collections import Counter
import csv
import json
import math
from pathlib import Path


def read_rows(file):
    if not file.exists():
        return []
    return [json.loads(line) for line in file.read_text().splitlines() if line]


def host_for(point, spawn, boxes):
    if point is None:
        return None
    c,s=math.cos(spawn['yaw']),math.sin(spawn['yaw'])
    x=spawn['x']+c*point[0]-s*point[1]
    y=spawn['y']+s*point[0]+c*point[1]
    matches=[]
    for box in boxes:
        dx,dy=x-box['x'],y-box['y']
        c,s=math.cos(box['yaw']),math.sin(box['yaw'])
        u,v=abs(c*dx+s*dy),abs(-s*dx+c*dy)
        distance=math.hypot(max(0.,u-box['sx']/2),max(0.,v-box['sy']/2))
        if distance<=.35:
            matches.append(box['name'])
    return matches[0] if len(matches)==1 else None


def coverage_changes(path, intervals):
    if not path.is_file():
        return None
    with path.open() as stream:
        samples = [(float(r['mission_elapsed_s']),int(r['confirmed_boxes']),int(r['observed_faces']))
                   for r in csv.DictReader(stream)]
    times = [s[0] for s in samples]
    if any(b<a for a,b in zip(times,times[1:])):
        raise ValueError('Coverage trace clock moved backwards')
    counts=Counter()
    for item in intervals:
        start=item['start_s']
        end=start+item['duration_s']
        first=bisect.bisect_right(times,start)-1
        last=bisect.bisect_right(times,end)-1
        if first<0 or last<=first or start-times[first]>1.5 or end-times[last]>1.5:
            counts['unmeasured_intervals']+=1
            continue
        span=samples[first:last+1]
        if any(b[0]-a[0]>1.5 or b[1]<a[1] or b[2]<a[2] for a,b in zip(span,span[1:])):
            counts['unmeasured_intervals']+=1
            continue
        boxes=span[-1][1]-span[0][1]
        faces=span[-1][2]-span[0][2]
        item.update(newly_confirmed_boxes=boxes,reported_face_increase=faces)
        # When an object becomes confirmed, faces observed before this interval
        # can enter the reported total at once. Separate those intervals.
        if boxes:
            counts['intervals_with_new_confirmation']+=1
        else:
            counts['intervals_without_new_confirmation']+=1
            counts['face_increase_without_new_confirmation']+=faces
            counts['zero_gain_without_new_confirmation']+=int(faces==0)
    return dict(samples=len(samples),**counts)


def summarize(folder):
    audit=json.loads((folder/'audit.json').read_text())
    scene=json.loads((folder/'scene.json').read_text())
    start=audit['simulation_start_clock']
    end=start+audit['elapsed_sim_s']
    goals=read_rows(folder/'native/hybrid_goals.jsonl')
    selected=[r for r in read_rows(folder/'native/marker_directed_looks.jsonl')
              if r['event']=='selected' and r.get('kind')=='passed_box']
    stamps=[r['stamp'] for r in selected]
    expected={t['attached_to']:str(t['id']) for t in scene['targets']}
    host_seconds=Counter()
    counts=Counter()
    intervals=[]
    for i,g in enumerate(goals):
        if g['kind'] not in ('box_face_view','box_look') or not start<=g['t']<end:
            continue
        finish=min(goals[i+1]['t'],end) if i+1<len(goals) else end
        if finish<g['t']:
            raise ValueError('Goal timestamps are not ordered')
        # Separate files share the same ROS clock. Count unmatched/ambiguous
        # events explicitly instead of guessing the intended object.
        lo=bisect.bisect_left(stamps,g['t']-.25)
        hi=bisect.bisect_right(stamps,g['t']+.25)
        point=selected[lo].get('point_odom') if hi-lo==1 else None
        host=host_for(point,scene['robot_spawn'],scene['obstacles'])
        ident=expected.get(host)
        registered=audit['registered_s'].get(ident)
        category=('unmatched' if host is None else
                  'unmarked_box' if ident is None else
                  'previously_registered_box' if registered is not None and registered<=g['t']-start
                  else 'unregistered_marked_box')
        duration=finish-g['t']
        host_seconds[category]+=duration
        counts[category]+=1
        intervals.append(dict(host=host,id=ident,start_s=round(g['t']-start,3),duration_s=round(duration,3),
                              classification_at_dispatch=category))
    registered=set(audit['registered_s'])
    truth={str(t['id']) for t in scene['targets']}
    saved=set(audit['saved_map_xy'])
    trace=coverage_changes(folder/'box_coverage.csv',intervals)
    return dict(run=folder.name,coverage_changes=trace,unexpected_registered_ids=sorted(registered-truth),
                missing_saved_ids=sorted(truth-saved),unexpected_saved_ids=sorted(saved-truth),
                goals=dict(counts),seconds={k:round(v,3) for k,v in host_seconds.items()},
                intervals=intervals)


if __name__=='__main__':
    import sys
    print(json.dumps([summarize(Path(arg)) for arg in sys.argv[1:]],indent=2))
