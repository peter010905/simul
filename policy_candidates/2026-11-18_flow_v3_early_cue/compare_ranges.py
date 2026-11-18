"""Replay identical calibrated frames through near-only and near-plus-far cue tracking."""
import argparse
import json
import sys
import tempfile
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import cv2
import numpy as np
from far_face import far_face
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'2026-11-18_flow_v3_preview_bearing'))
from replay_bearing import preview_geometry
from replay_recovery import borders,plane_hit,can_confirm
from compare import SquarePriority,decode,local_observations,replay


def evaluate(args):
    run,out=args
    cv2.setNumThreads(1)
    start=next(e['wall_epoch_s'] for e in map(json.loads,(run/'artifacts/startup.jsonl').read_text().splitlines()) if e['event']=='mission_started')
    states=SquarePriority(),SquarePriority()
    refs,reads,trace={},{},[]
    dic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    def reference(mid,t,point,base):
        refs.setdefault(int(mid),(t,point,np.array(base[:2])))
    def frame(t,stamp,scan,o,cam,cal,img):
        g=preview_geometry(cal)
        if g is cal:raise ValueError('unexpected camera calibration')
        quads,ids=decode(img,dic)
        decoded=[]
        for q,i in zip(quads,[] if ids is None else ids.flatten()):
            edge=float(np.linalg.norm(q[0]-np.roll(q[0],1,axis=0),axis=1).min())
            if edge>=18:
                reads.setdefault(int(i),t)
                decoded.append([int(i),q[0].mean(axis=0).tolist()])
        candidates=borders(img)
        samples=list(local_observations(scan,o,cam,g,img)) if candidates else []
        tracks=[[],[]];valid=[set(),set()];seen=[]
        for u,v,size in candidates:
            hit=plane_hit(u,size,samples,cam,g)
            new=hit if hit is not None else far_face(u,size,scan,o,cam,g)
            seen.append([round(u,1),round(v,1),round(size,1),hit is not None,new is not None])
            for idx,point in enumerate([hit,new]):
                if point is not None:
                    if can_confirm(u,size,img.shape[1]):valid[idx].add(len(tracks[idx]))
                    tracks[idx].append((*point,size))
        choices=[]
        for k,state in enumerate(states):
            prev=len(state.looks)
            state.track_frame(tracks[k],stamp,t,cam,confirmable=valid[k])
            choice=None
            if len(state.looks)>prev:
                look=state.looks[-1]
                choice=dict(t=round(t-start,3),xy=look['point'].tolist(),action=look['action'])
            choices.append(choice)
        trace.append(dict(t=round(t-start,3),candidates=seen,choices=choices,decoded=decoded))
    # Immediate camera requests do not consume saved navigation decisions.
    with tempfile.TemporaryDirectory() as tmp:
        local=Path(tmp)/run.name
        (local/'artifacts').mkdir(parents=True)
        for path in ['rosbag','artifacts/startup.jsonl']:(local/path).symlink_to(run/path)
        replay(local,Path(tmp),record_reference=reference,inspect_frame=frame)
    markers={}
    for mid,(t,p,b) in refs.items():
        cutoff=min(t,reads.get(mid,t))
        both=[]
        for state in states:
            pre=[x for x in state.looks if x['t']<cutoff]
            hits=[x for x in pre if np.linalg.norm(x['point']-p)<=.30 and x['normal']@(b-x['point'])>0]
            both.append(dict(first_match=bool(hits and hits[0] is pre[0]),any_match=bool(hits),count=len(pre),
                             matching_t=round(hits[0]['t']-start,3) if hits else None,
                             first_distance=float(np.linalg.norm(pre[0]['point']-p)) if pre else None))
        markers[mid]=dict(cutoff=round(cutoff-start,3),first_video=round(reads[mid]-start,3) if mid in reads else None,
                          recorded=round(t-start,3),old=both[0],new=both[1])
    (out/(run.name+'-frames.json')).write_text(json.dumps(trace,separators=(',',':')))
    result=dict(run=run.name,frames=len(trace),requests=[len(s.looks) for s in states],markers=markers)
    print(json.dumps(result),flush=True)
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path);parser.add_argument('out',type=Path)
    opts=parser.parse_args();opts.out.mkdir(parents=True,exist_ok=True)
    split=json.loads((HERE/'split.json').read_text())
    names=split['development']+split['evaluation']
    with ProcessPoolExecutor(max_workers=2) as pool:
        rows=list(pool.map(evaluate,[(opts.root/n,opts.out) for n in names]))
    (opts.out/'results.json').write_text(json.dumps(rows,indent=2)+'\n')
