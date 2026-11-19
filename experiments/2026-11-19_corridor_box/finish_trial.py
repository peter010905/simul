"""Verify truth and saved IDs, encode the recorded RGB frames, then audit visible IDs."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

def finish(root, folder):
    result=json.loads((folder/'metrics.json').read_text())
    scene=yaml.safe_load((root/'scene.yaml').read_text())
    targets={t['id']:(t['x'],t['y']) for t in scene['targets']}
    events=[json.loads(l) for l in (folder/'events.jsonl').read_text().splitlines()]
    start=next(e['sim'] for e in events if e['event']=='start_ack')
    with (folder/'trajectory.csv').open() as f:
        rows=list(csv.DictReader(f))[:result['truth_samples']]
    mins={ident:min(math.dist((float(r['world_x']),float(r['world_y'])),point) for r in rows) for ident,point in targets.items()}
    assert all(abs(result['min_ground_truth_m'][str(i)]-d)<1.e-6 for i,d in mins.items())
    assert result['bumper_events']==sum(e['event']=='bumper' and e['sim']<=start+result['elapsed_sim_s']+1e-3 for e in events)
    saved={}
    files=list((folder/'native').glob('*/markers.csv'))
    if files:
        with files[0].open() as f:
            saved={int(r['id']):[float(r['x']),float(r['y'])] for r in csv.DictReader(f)}
    with (folder/'frames.csv').open() as f:
        frames=list(csv.DictReader(f))
    assert len(frames)==result['camera_frames']
    stamps=np.array([float(r['elapsed_sim_s']) for r in frames])
    assert np.all(np.diff(stamps)>0) and stamps[0]<1 and abs(result['elapsed_sim_s']-stamps[-1])<1.
    # Resample against image header time; a missing camera frame holds the last actual image.
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    decodes={i:[] for i in targets}
    first=cv2.imread(str(folder/'camera/00001.jpg')); height,width=first.shape[:2]
    temp=folder/'mission.tmp.mp4'
    writer=cv2.VideoWriter(str(temp),cv2.VideoWriter_fourcc(*'avc1'),5.,(width,height))
    if not writer.isOpened(): raise RuntimeError('H.264 encoder unavailable')
    count=max(1,round(result['elapsed_sim_s']*5))
    indices=np.maximum(0,np.searchsorted(stamps,np.arange(count)/5,side='right')-1)
    for i,frame in enumerate(frames):
        img=cv2.imread(str(folder/'camera'/f'{int(frame["frame"]):05d}.jpg'))
        corners,ids,_=cv2.aruco.detectMarkers(img,dictionary)
        if ids is not None:
            for corner,ident in zip(corners,ids.flatten()):
                if int(ident) in decodes:
                    decodes[int(ident)].append(dict(t=round(stamps[i],3),center_x=round(float(corner[...,0].mean()/width),3)))
        for _ in range(int(np.count_nonzero(indices==i))):
            writer.write(img)
    writer.release()
    cap=cv2.VideoCapture(str(temp))
    assert int(cap.get(cv2.CAP_PROP_FRAME_COUNT))==count and cap.read()[0]
    cap.set(cv2.CAP_PROP_POS_FRAMES,count-1);assert cap.read()[0];cap.release()
    temp.replace(folder/'mission.mp4')
    r={k:v for k,v in result.items() if k!='last_policy_status'}
    choices=[]
    hybrid=folder/'native/hybrid_goals.jsonl'
    if hybrid.exists():choices=[json.loads(l)['kind'] for l in hybrid.read_text().splitlines()]
    r.update(run=folder.name,saved_map_xy=saved,simulation_start_clock=start,
             video=dict(frames=count,duration_s=count/5,recorded_frames=len(frames),max_frame_gap_s=float(np.diff(stamps).max()),
                        sha256=hashlib.sha256((folder/'mission.mp4').read_bytes()).hexdigest()),hybrid_goal_types=choices,
             offline_rgb={k:dict(frames=len(v),first=v[0] if v else None,last=v[-1] if v else None) for k,v in decodes.items()})
    (folder/'audit.json').write_text(json.dumps(r,indent=2))
    return r

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('root',type=Path)
    parser.add_argument('folder',type=Path)
    args=parser.parse_args()
    cv2.setNumThreads(1)
    r=finish(args.root,args.folder)
    print(r['run'],r['final_ids'],r['registered_and_within_s'],r['video']['duration_s'],flush=True)
