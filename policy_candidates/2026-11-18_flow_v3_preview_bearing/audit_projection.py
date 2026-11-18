"""Trace both projection models on the same selected contour and ROS messages."""
import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
from replay_bearing import preview_geometry
from compare import replay, local_observations
from replay import header_time
from recovery import borders, plane_hit


def inspect(run, lo, hi, ident):
    start=next(e['wall_epoch_s'] for e in map(json.loads,(run/'artifacts/startup.jsonl').read_text().splitlines())
               if e['event']=='mission_started')
    references=[]
    frames=[]
    def frame(t, stamp, scan, origin, camera, cal, image):
        if not lo<=t-start<=hi:return
        for u,v,size in borders(image):
            row=dict(time_s=round(t-start,4),image_stamp_s=round(stamp-start,4),
                     lidar_stamp_s=round(header_time(scan)-start,4),u=u,v=v,
                     camera_xy=list(camera[:2]),camera_yaw_deg=math.degrees(camera[2]))
            undist=cv2.undistortPoints(np.float64([[[u,v]]]),np.array(cal.k).reshape(3,3),np.array(cal.d))
            row['raw_undistorted_bearing_deg']=-math.degrees(math.atan(undist[0,0,0]))
            for name,g in [('original',cal),('corrected',preview_geometry(cal))]:
                samples=list(local_observations(scan,origin,camera,g,image))
                hit=plane_hit(u,size,samples,camera,g)
                angle=-math.atan((u-g.k[2])/g.k[0])
                row[name]=dict(relative_deg=math.degrees(angle),world_deg=math.degrees(angle+camera[2]),
                               xy=hit[0].tolist() if hit else None,
                               range_m=float(np.linalg.norm(hit[0]-camera[:2])) if hit else None)
            frames.append(row)
    def reference(mid,t,point,base):
        if mid==ident:references.append(dict(t=t-start,xy=point.tolist()))
    # replay() emits a detailed per-run file; keep that outside the project.
    import tempfile
    with tempfile.TemporaryDirectory() as folder:
        replay(run,Path(folder),inspect_frame=frame,record_reference=reference)
    return dict(frames=frames,first_logged_observation=references[0])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path);p.add_argument('start',type=float);p.add_argument('end',type=float)
    p.add_argument('id',type=int);p.add_argument('output',type=Path)
    args=p.parse_args()
    args.output.write_text(json.dumps(inspect(args.run,args.start,args.end,args.id),indent=2)+'\n')
