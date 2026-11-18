"""Paired frozen-log replay. Decode and recorded references are evaluation-only."""
import argparse
import json
import math
import sys
import tempfile
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import cv2
import numpy as np
import yaml

from glance import EdgeGlance
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'2026-11-18_flow_v3_early_cue'))
from compare_ranges import SquarePriority,borders,plane_hit,far_face,can_confirm,local_observations,preview_geometry,replay,decode
from fit_preview import stream,INFO


def mission(run):
    f=run/'artifacts/startup.jsonl'
    events=[json.loads(line) for line in f.read_text().splitlines()]
    start=next((e['wall_epoch_s'] for e in events if e['event']=='mission_started'),None)
    return start if start and any(e['event'] in ('stopped','safety_stop','mission_finished') for e in events) else None


def score(args):
    run,group,out=args
    cv2.setNumThreads(1)
    info=next(stream(run,[INFO]),(None,None,None))[2]
    start=mission(run)
    if start is None or info is None or preview_geometry(info) is info:
        return dict(run=run.name,group=group,skip='mission bounds or camera profile')
    folder=out/run.name;folder.mkdir(exist_ok=True)
    baseline=SquarePriority();overlay=EdgeGlance()
    refs={};reads={};events=[];frames=0
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    def reference(mid,t,point,base):
        refs.setdefault(int(mid),dict(t=t-start,xy=point.tolist(),observer=list(base[:2])))
    def frame(t,stamp,scan,o,cam,cal,img):
        nonlocal frames
        frames+=1
        g=preview_geometry(cal)
        candidates=borders(img)
        samples=list(local_observations(scan,o,cam,g,img)) if candidates else []
        tracks=[];can=set();attached=[]
        for u,v,s in candidates:
            hit=plane_hit(u,s,samples,cam,g)
            hit=hit if hit is not None else far_face(u,s,scan,o,cam,g)
            attached.append(hit)
            if hit is None:continue
            if can_confirm(u,s,img.shape[1]):can.add(len(tracks))
            tracks.append((*hit,s))
        n=len(baseline.looks)
        baseline.track_frame(tracks,stamp,t,cam,confirmable=can)
        confirmed=None
        if len(baseline.looks)>n:
            look=baseline.looks[-1]
            i=min((i for i,h in enumerate(attached) if h is not None),
                  key=lambda i:np.linalg.norm(attached[i][0]-look['point']))
            u,v,s=candidates[i]
            confirmed=dict(action=look['action'],source='confirmed',uv=[u,v],size=s,
                           yaw_delta_deg=look['yaw_delta_deg'],dwell_s=look['dwell_s'],
                           xy=look['point'].tolist(),normal=look['normal'].tolist())
        candidate=overlay.update(t,stamp,cam,img.shape[1],g.k[0],g.k[2],candidates,confirmed)
        if candidate and candidate['source']=='single':
            i=next(i for i,p in enumerate(candidates) if list(p[:2])==candidate['uv'])
            h=attached[i]
            candidate.update(xy=h[0].tolist() if h else None,normal=h[1].tolist() if h else None)
        sec=t-start
        if confirmed or candidate:
            events.append(dict(t=sec,baseline=confirmed,candidate=candidate))
            cv2.imwrite(str(folder/f'{sec:.6f}.png'),img)
        qs,ids=decode(img,dictionary)
        for q,mid in zip(qs,[] if ids is None else ids.ravel()):
            if np.linalg.norm(q[0]-np.roll(q[0],1,axis=0),axis=1).min()>=18:
                mid=int(mid)
                if mid not in reads:
                    reads[mid]=dict(t=sec,uv=q[0].mean(axis=0).tolist())
                    cv2.imwrite(str(folder/f'id{mid}.png'),img)
    with tempfile.TemporaryDirectory() as tmp:
        local=Path(tmp)/run.name
        (local/'artifacts').mkdir(parents=True)
        for path in ['rosbag','artifacts/startup.jsonl']:(local/path).symlink_to(run/path)
        replay(local,Path(tmp),record_reference=reference,inspect_frame=frame)
    r=dict(run=run.name,group=group,frames=frames,refs=refs,first_decode=reads,events=events)
    (folder/'summary.json').write_text(json.dumps(r))
    return r


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);p.add_argument('out',type=Path)
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    plan=json.loads((HERE/'cohorts.json').read_text());jobs=[]
    for meta in sorted(a.root.glob('*/rosbag/metadata.yaml')):
        run=meta.parents[1];key=run.name[:15]
        group='previously_reviewed' if key in plan['previously_reviewed'] else 'unseen'
        if group=='unseen' and key in plan['exclude']:continue
        rows=yaml.safe_load(meta.read_text())['rosbag2_bagfile_information']['topics_with_message_count']
        c={r['topic_metadata']['name']:r['message_count'] for r in rows}
        if c.get('/oakd/rgb/preview/image_raw',0)<100 or c.get('/scan_self_filtered',0)==0:continue
        jobs.append((run,group,a.out))
    with ProcessPoolExecutor(max_workers=2) as pool:results=list(pool.map(score,jobs))
    (a.out/'results.json').write_text(json.dumps(results))
