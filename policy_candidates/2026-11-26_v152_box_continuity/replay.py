"""One-step counterfactuals at recorded face interruptions. Retain the historical route."""
from collections import Counter
import json
from pathlib import Path
from types import SimpleNamespace
import time

import numpy as np
from osp_mission.center_wall import CenterWall
from osp_mission.box_continuity import BoxContinuity
from osp_mission.hybrid import Hybrid, PrintMemory
from osp_mission._r32_navigation_core import Grid, travel_distances
from osp_mission.box_tour import short_tour
from scipy.ndimage import distance_transform_edt


def records(path):
    return [json.loads(s) for s in path.read_text().splitlines()]


run=Path.home()/'simul/run_logs/robot/20261125T174857-2f02f6e167'
root=run/'artifacts'
rows=records(root/'center_wall.jsonl')
goals=records(root/'hybrid_goals.jsonl')
face_events=records(root/'face_directed_looks.jsonl')
base=goals[0]['t']-rows[0]['elapsed']
checks=[]
for i,event in enumerate(face_events):
    if event['event']!='selected':
        continue
    row=min(rows,key=lambda r:abs(r['elapsed']+base-event['stamp']))
    if abs(row['elapsed']+base-event['stamp'])>.10 or row['kind']!='interior_surface':
        continue
    ending=next((x for x in face_events[i+1:] if x['event'] in ('cleared','consumed')),None)
    if ending is None or ending.get('reason')!='new_marker_hint':
        continue
    next_choice=next((r for r in rows if r['elapsed']+base>ending['stamp']),None)
    if next_choice is None or not next_choice['white_leg']:
        continue
    # The track is the already-selected historical white cue. Do not reinterpret it as an ID.
    elapsed=row['elapsed']
    g=dict(r32_kind='interior_surface',face_inspection=True,surface_odom=[0.,0.],
           normal_odom=0.,batch_box=row['batch_box'],region_face=tuple(row['region_face']) if row['region_face'] else None,
           region_anchor=[0.,0.],previous_attempts=0,completed_box=False)
    h=Hybrid()
    h.schedule=CenterWall()
    h.schedule.dispatched(g,elapsed)
    clock=ending['stamp']-base
    owner=SimpleNamespace(now=lambda:clock,started=0.,get_clock=lambda:SimpleNamespace(
        now=lambda:SimpleNamespace(nanoseconds=int(clock*1e9))))
    h.memory.tracks=[dict(count=3,stamp=clock-.01,white_leg=True)]
    white_allowed=h.has_fresh_print(owner)
    h.memory.tracks.append(dict(count=3,stamp=clock-.01,white_leg=False))
    named_or_pattern_allowed=h.has_fresh_print(owner)
    # Replay the interruption under the new ledger even if caused by a strong marker.
    h.schedule.face_result('nav',5,'marker_hint',clock)
    grid=Grid(np.zeros((80,80),np.int8),.1,(-4.,-4.,0.))
    safe=np.ones((80,80),bool)
    resumed=h.schedule.continuity.choose([(1.,dict(g,x=1.,y=1.))],clock+.5,100.,grid,safe)
    checks.append(dict(face_selected_s=round(elapsed,3),interrupted_s=round(clock,3),
                       next_historical_reason=next_choice['reason'],white_interrupt_allowed=white_allowed,
                       stronger_pattern_interrupt_allowed=named_or_pattern_allowed,
                       interrupted_history_count=len(h.schedule.history),
                       resume_available=resumed is not None))
# Test ordering on 64 individual legs. This layout is synthetic, not reconstructed physical truth.
cells=np.zeros((120,120),np.int8)
for r in range(8):
    for c in range(8):
        cells[15+r*10:17+r*10,15+c*10:17+c*10]=100
grid=Grid(cells,.1,(0.,0.,0.))
safe=(cells==0)&(distance_transform_edt(np.pad(cells==0,1))[1:-1,1:-1]*.1>=.32)
distances=travel_distances(safe,*grid.index(.6,5.5),.1)
points=[(1.,1.),(10.5,1.),(10.5,10.5),(1.,10.5)]
options=[(float(distances[grid.index(*p)]),dict(x=p[0],y=p[1],normal_odom=i*np.pi/2)) for i,p in enumerate(points)]
times=[]
for _ in range(5):
    start=time.perf_counter(); result=short_tour(options,grid,safe,300.)
    times.append((time.perf_counter()-start)*1000)
out=dict(run=run.name,scope='historical interruption timing and ledger counterfactual; recorded trajectory unchanged; resume regeneration tested on synthetic safe grid',
         historical_goals=len(rows),historical_white_goals=sum(r['white_leg'] for r in rows),
         interior_white_interruptions_checked=len(checks),checks=checks,
         synthetic_64_legs=dict(faces=result['box_tour_faces'],walk_m=result['box_tour_walk_m'],
                               processing_ms_median=float(np.median(times)),observed_motion=False))
assert checks and all(not r['white_interrupt_allowed'] and r['stronger_pattern_interrupt_allowed']
                      and r['interrupted_history_count']==0 and r['resume_available'] for r in checks)
print(json.dumps(out,indent=2))
