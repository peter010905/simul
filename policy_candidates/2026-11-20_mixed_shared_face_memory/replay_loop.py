"""Replay the generic surface filter on recorded decision grids and reconstructed camera exposure."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from osp_mission._r32_navigation_core import Grid
from osp_mission.box_views import local_boxes, odom_point
from osp_mission.center_wall import surface_cells
from osp_mission.edge_policy import DIRECTIONS
from osp_mission.tracked_faces import TrackedFaces


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def closest(times, stamp):
    i = int(np.searchsorted(times, stamp))
    return min(range(max(0,i-1), min(len(times),i+1)),key=lambda j:abs(times[j]-stamp))


def observed(faces, grid, transform, camera):
    # Only previously recorded image times. Robot poses align world and SLAM;
    # no scene geometry, target IDs, final trajectories or object labels are used by the policy.
    seen = []
    rot = transform[2]
    c,s = math.cos(rot), math.sin(rot)
    for face, normal in faces:
        anchor = odom_point(face,transform)
        normal_yaw = math.atan2(normal[1],normal[0]) + rot
        delta = camera[:,1:3] - anchor
        dist = np.linalg.norm(delta, axis=1)
        bearing = np.arctan2(-delta[:,1],-delta[:,0])
        good = ((dist >= .10) & (dist <= 1.5) &
                ((delta @ np.array([math.cos(normal_yaw),math.sin(normal_yaw)])) >= .65*dist) &
                (np.cos(bearing-camera[:,3]) >= math.cos(math.radians(25))))
        previous = -math.inf
        for i in np.flatnonzero(good):
            t,x,y,_ = camera[i]
            dx,dy=x-transform[0],y-transform[1]
            eye = (c*dx+s*dy, -s*dx+c*dy)
            if not grid.line_clear(eye,face+.12*normal):
                continue
            if t-previous > .32:
                first=t
            if t-first >= .4:
                seen.append(normal_yaw)
                break
            previous=t
    return seen


def replay(run, first_goal, last_goal):
    schedules = [json.loads(s) for s in (run/'native/center_wall.jsonl').read_text().splitlines()]
    headings = np.array([[float(q[k]) for k in ('elapsed_sim_s','x','y','yaw_rad')]
                         for q in rows(run/'headings.csv')])
    times = headings[:,0]
    stamps = [float(r['elapsed_sim_s']) for r in rows(run/'frames.csv')]
    camera = np.array([[t,*headings[closest(times,t),1:]] for t in stamps])
    memory=TrackedFaces()
    confirmed=[]
    audit=[]
    for snap in sorted((run/'native/policy_decisions').glob('*.npz')):
        with np.load(snap,allow_pickle=False) as z:
            t=float(z['elapsed'])
            number=min(range(len(schedules)),key=lambda i:abs(schedules[i]['elapsed']-t))+1
            rec=schedules[number-1]
            if abs(rec['elapsed']-t)>.15 or number>last_goal: continue
            pose=(*z['robot'],float(z['heading']))
            grid=Grid(z['data'],float(z['resolution']),(*z['origin'][:2],0.))
            coverage=z['viewed']
        j=closest(times,t)
        _,x,y,yaw=headings[j]
        a=yaw-pose[2];c,s=math.cos(a),math.sin(a)
        transform=(x-c*pose[0]+s*pose[1],y-s*pose[0]-c*pose[1],a)
        fitted=local_boxes(grid,pose)
        # At this time the original run had demonstrated an actual confirmed rectangle.
        if rec.get('fitted_face') and fitted:
            side=rec['fitted_face']-1
            obj,faces=min(fitted,key=lambda item:np.linalg.norm(item[1][side][0]-rec['surface']))
            if np.linalg.norm(faces[side][0]-rec['surface']) < .16:
                anchor=odom_point(obj,transform)
                if all(np.linalg.norm(anchor-p)>.4 for p in confirmed):
                    confirmed.append(anchor)
        before=camera[camera[:,0]<=t]
        current=[(center,faces,observed(faces,grid,transform,before)) for center,faces in fitted
                 if any(np.linalg.norm(odom_point(center,transform)-p)<.4 for p in confirmed)]
        memory.update(current,transform,t)
        if number<first_goal or rec['kind']!='interior_surface' or rec.get('fitted_face'):continue
        options=[]
        for (kind,_,side,_,_),cells in surface_cells(grid,pose).items():
            if kind!='interior':continue
            remaining=[(r,k) for r,k in cells if not coverage[side,r,k]]
            if len(remaining)*grid.resolution<.15:continue
            dy,dx=DIRECTIONS[side]
            r,k=remaining[len(remaining)//2]
            mid=grid.world(r+.5*dy,k+.5*dx)
            options.append((float(np.linalg.norm(mid-rec['surface'])),side,remaining,mid))
        if not options:
            result=dict(goal=number,status='bin_unavailable')
        else:
            err,side,remaining,mid=min(options,key=lambda v:v[0])
            dy,dx=DIRECTIONS[side]
            angle=math.atan2(dy,dx)+a
            owner=memory.match(odom_point(mid,transform),angle,grid.resolution)
            replacement=memory.representative(grid,remaining,dy,dx,transform,angle)
            if err > 1.e-4: status='bin_unmatched'
            elif replacement is None: status='rejected_observed'
            elif np.linalg.norm(replacement[0]-mid)>.001: status='redirected_within_bin'
            else: status='unchanged'
            result=dict(goal=number,stamp=t,status=status,bin_match_error_m=err,
                        old_fragment_owned=owner is not None,old_face_observed=bool(owner and owner.observed),
                        seen_count_at_current_fits=[len(a[2]) for a in current])
        audit.append(result)
    return audit


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('run',type=Path)
    parser.add_argument('--first',type=int,default=6)
    parser.add_argument('--last',type=int,default=13)
    p=parser.parse_args()
    print(json.dumps(replay(p.run,p.first,p.last),indent=2))
