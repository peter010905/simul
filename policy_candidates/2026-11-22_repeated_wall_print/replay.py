"""Offline wall relook requests from recorded camera frames and past SLAM snapshots."""
import argparse
import csv
import json
from bisect import bisect_right
from collections import defaultdict, deque
from pathlib import Path

import cv2
import numpy as np


def wall_hit(grid, origin, cell, eye, ray):
    lengths = np.arange(.3, 18., cell / 2)
    xy = eye + lengths[:, None] * ray
    ij = np.floor((xy - origin) / cell).astype(int)
    h, w = grid.shape
    valid = (ij[:, 0] >= 0) & (ij[:, 0] < w) & (ij[:, 1] >= 0) & (ij[:, 1] < h)
    # Stop at the map edge. A ray must originate in known mapped space.
    if not valid[0]:
        return None
    end = next(iter(np.flatnonzero(~valid)), len(ij))
    ij = ij[:end]
    values = grid[ij[:, 1], ij[:, 0]]
    occupied = np.flatnonzero(values >= 65)
    if not len(occupied):
        return None
    k = occupied[0]
    if np.any(values[:k] < 0):
        return None
    x, y = ij[k]
    radius = int(np.ceil(1.0 / cell))
    lo_x, lo_y = max(0, x-radius), max(0, y-radius)
    rows, cols = np.where(grid[lo_y:y+radius+1, lo_x:x+radius+1] >= 65)
    offsets = np.column_stack([cols+lo_x-x, rows+lo_y-y]) * cell
    points = offsets[np.linalg.norm(offsets,axis=1) <= 1.0]
    if len(points) < 6:
        return None
    _, _, axes = np.linalg.svd(points - points.mean(axis=0), full_matrices=False)
    if np.ptp(points @ axes[0]) < 1.25 or np.percentile(abs(points @ axes[1]), 90) > .15:
        return None
    return xy[k]


class RepeatEvidence:
    def __init__(self):
        self.by_id = defaultdict(deque)
        self.sent = defaultdict(list)

    def observe(self, ident, stamp, hit):
        q = self.by_id[ident]
        while q and stamp-q[0][0] > 1.0:
            q.popleft()
        if any(t == stamp for t, _ in q):
            return None
        q.append((stamp, hit))
        close = [p for _, p in q if np.linalg.norm(hit-p) <= .5]
        if len(close) < 3:
            return None
        point = np.median(close, axis=0)
        if any(np.linalg.norm(point-old) <= .8 for old in self.sent[ident]):
            return None
        self.sent[ident].append(point)
        return point


def replay(folder, bag):
    from range_replay import recorded_transforms
    metrics = json.loads((folder/'metrics.json').read_text())
    start, = [e['sim'] for e in map(json.loads,(folder/'events.jsonl').read_text().splitlines()) if e['event']=='start_ack']
    with (folder/'frames.csv').open() as f:
        frames=[(int(r['frame']),float(r['elapsed_sim_s'])) for r in csv.DictReader(f)
                if 0 <= float(r['elapsed_sim_s']) <= min(600.,metrics['elapsed_sim_s'])]
    mask,K,D,poses,fixes,_,_=recorded_transforms(bag,np.array([start+t for _,t in frames]))
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    maps=sorted((float(np.load(p)['elapsed']),p) for p in (folder/'native/policy_decisions').glob('*.npz'))
    times=[t for t,_ in maps]
    selected=-1
    evidence=RepeatEvidence()
    hints=[]
    looks=[]
    bearing_history=defaultdict(deque)
    last_look={}
    raw=hits=0
    registered=metrics['registered_s']
    for (n,t),pose,fix in zip((v for v,ok in zip(frames,mask) if ok), poses, fixes):
        idx=bisect_right(times,t)-1
        if idx < 0:
            continue
        img=cv2.imread(str(folder/'camera'/f'{n:05d}.jpg'),cv2.IMREAD_GRAYSCALE)
        if img is None: raise ValueError(f'Missing frame {n}')
        quads, ids, _=cv2.aruco.detectMarkers(img,dic)
        if ids is None: continue
        for ident,quad in zip(ids.ravel(),quads):
            ident=int(ident)
            quad=quad.reshape(4,2)
            pixels=float(np.min(np.linalg.norm(quad-np.roll(quad,1,axis=0),axis=1)))
            if not 5 <= pixels < 18 or registered.get(str(ident),float('inf')) <= t: continue
            raw+=1
            if idx!=selected:
                with np.load(maps[idx][1]) as snap:
                    grid,origin,cell=snap['data'],snap['origin'],float(snap['resolution'])
                selected=idx
            uv=cv2.undistortPoints(quad.mean(axis=0).reshape(1,1,2),K,D).ravel()
            cam=fix@pose
            ray=(cam[:3,:3]@np.r_[uv,1.])[:2]
            ray=ray/np.linalg.norm(ray)
            # Without a mapped wall, retain only a short camera turn in place.
            # Compare in odom so SLAM corrections cannot create extra votes.
            odom_ray=(pose[:3,:3]@np.r_[uv,1.])[:2]
            odom_ray/=np.linalg.norm(odom_ray)
            recent=bearing_history[ident]
            while recent and t-recent[0][0]>1.0: recent.popleft()
            if not recent or recent[-1][0]!=t:
                recent.append((t,odom_ray.copy(),pose[:2,3].copy()))
            agreeing=[v for v in recent if v[1]@odom_ray>=np.cos(np.deg2rad(5))
                      and np.linalg.norm(pose[:2,3]-v[2])<=.35]
            if len(agreeing)>=3 and t-last_look.get(ident,-1e6)>90:
                looks.append(dict(id=ident,t=t,eye=cam[:2,3].tolist(),ray=ray.tolist()))
                last_look[ident]=t
            hit=wall_hit(grid,origin,cell,cam[:2,3],ray)
            if hit is None: continue
            hits+=1
            point=evidence.observe(ident,t,hit)
            if point is not None: hints.append(dict(id=ident,t=t,xy=point.tolist(),pixels=pixels))
    # Compare against recorded decisions. This is a counterfactual request, never a driven goal.
    decisions=[json.loads(s) for s in (folder/'native/center_wall.jsonl').read_text().splitlines()]
    for h in hints:
        h['request_s']=next((d['elapsed'] for d in decisions if h['t']<=d['elapsed']<=h['t']+90
                              and d['elapsed']<registered.get(str(h['id']),float('inf'))
                              and d.get('kind') in ('wall_surface','interior_frontier','frontier')),None)
    # Ground truth is used only after the entire selection sequence has been computed.
    align=metrics['world_from_map_initial_alignment']
    gt={m['id']:m for m in json.loads((folder/'scene.json').read_text())['targets']}
    for h in looks:
        m=gt.get(h['id'])
        h['host']=m['attached_to'] if m else None
        if m:
            R=np.array(align['rotation'])
            eye=R@h['eye']+np.array(align['translation'])
            to_target=np.array([m['x'],m['y']])-eye
            cos_angle=float(np.clip((R@h['ray'])@to_target/np.linalg.norm(to_target),-1,1))
            h['error_deg']=float(np.degrees(np.arccos(cos_angle)))
        h['correct_direction']=bool(m and h['error_deg']<=5.)
    for h in hints:
        world=np.array(align['rotation'])@h['xy']+np.array(align['translation'])
        m=gt.get(h['id'])
        err=float(np.linalg.norm(world-[m['x'],m['y']])) if m else None
        h.update(wall_match=bool(m and m['attached_to'].startswith('wall_') and err<=.8),location_error_m=err)
    return dict(run=folder.name,recorded_frames=len(frames),frames_with_calibration=int(mask.sum()),
                raw_small_decodes_before_registration=raw,wall_hits=hits,hints=hints,bearing_only_looks=looks,
                native_distant_choices=sum(d.get('distant_hint_id') is not None for d in decisions))


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('recordings',type=Path)
    ap.add_argument('bags',type=Path)
    ap.add_argument('output',type=Path)
    a=ap.parse_args()
    rows=[]
    for bag in sorted(p for p in a.bags.iterdir() if p.is_dir()):
        rows.append(replay(a.recordings/bag.name,bag))
    a.output.write_text(json.dumps(rows,indent=2)+'\n')
