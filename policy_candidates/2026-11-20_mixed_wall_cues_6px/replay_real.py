"""Replay synchronized real RGB/camera poses and compare stable undriven bearing requests."""
import json
import math
from pathlib import Path
import sys
import cv2
from osp_mission.small_wall_cues import candidates
from osp_mission.wall_bearings import WallBearings

base,output=map(Path,sys.argv[1:3])
cv2.setNumThreads(1)
dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
summary=[]
for label in ('1737','1743','1723','1636','2116'):
    rows=[json.loads(line) for line in (base/(label+'.jsonl')).read_text().splitlines()]
    cap=cv2.VideoCapture(str(base/(label+'.avi')))
    trackers={6:WallBearings(),8:WallBearings()}
    props={6:0,8:0}
    events=[]
    for index,(t,stamp,pose,fx,cx) in enumerate(rows):
        ok,bgr=cap.read()
        assert ok
        gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
        q,ids,rej=cv2.aruco.detectMarkers(gray,dictionary)
        hits=candidates(gray,q,rej)
        if pose is None:
            continue
        for floor,tracker in trackers.items():
            for u,v,size,named in hits:
                if size < floor: continue
                props[floor]+=1
                a=pose[2]-math.atan2(u-cx,fx)
                tracker.observe(t,t,pose[:2],a,size)
            for track in tracker.tracks:
                if not track.stable() or getattr(track,'reported',False):
                    continue
                track.reported=True
                _,_,angle,_=track.samples[-1]
                u0=cx-fx*math.tan(angle-pose[2])
                if not hits:continue
                hit=min(hits,key=lambda p:abs(p[0]-u0))
                events.append(dict(floor=floor,t=t,index=index,u=hit[0],v=hit[1],size=hit[2]))
    cap.release()
    summary.append(dict(label=label,frames=len(rows),proposals=props,stable=events))
output.write_text(json.dumps(summary,indent=2))
