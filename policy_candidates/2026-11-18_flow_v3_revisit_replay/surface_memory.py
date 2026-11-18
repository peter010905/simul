"""Offline, odometry-anchored memory of the visible side of a physical surface."""
from collections import Counter, defaultdict
from dataclasses import dataclass, field
import math

import cv2
import numpy as np


def local_observations(scan, laser, camera, info, image):
    """One camera exposure and a synchronized planar lidar scan in the odom frame."""
    ranges = np.array(scan.ranges)
    angles = laser[2]+scan.angle_min+np.arange(len(ranges))*scan.angle_increment
    valid = np.isfinite(ranges) & (ranges>max(scan.range_min,.25)) & (ranges<min(scan.range_max,2.6))
    points = np.zeros((len(ranges),2))
    points[valid] = np.array(laser[:2]) + ranges[valid,None]*np.column_stack((np.cos(angles[valid]),np.sin(angles[valid])))
    sharpness = cv2.Laplacian(cv2.GaussianBlur(image,(3,3),.6),cv2.CV_32F)
    h,w = image.shape
    for i in range(2,len(ranges)-2,4):
        if not np.all(valid[i-2:i+3]):
            continue
        support=points[i-2:i+3]
        if np.linalg.norm(np.diff(support,axis=0),axis=1).max()>.14:
            continue
        edge=support[-1]-support[0]
        span=np.linalg.norm(edge)
        if span<.025:
            continue
        normal=np.array([-edge[1],edge[0]])/span
        if np.max(np.abs((support-points[i])@normal))>.025:
            continue
        dx,dy=points[i]-camera[:2]
        distance=math.hypot(dx,dy)
        if distance<.3:
            continue
        angle=math.atan2(dy,dx)-camera[2]
        if math.cos(angle)<=0:
            continue
        u=info.k[2]-info.k[0]*math.tan(angle)
        if not 6<=u<w-6:
            continue
        incidence=-(dx*normal[0]+dy*normal[1])/distance
        if incidence<0:
            incidence=-incidence
            normal=-normal
        if incidence<.22:
            continue
        # Use an image strip at this bearing, not a claim about marker height.
        center=int(round(u))
        area=np.s_[int(.2*h):int(.8*h), max(0,center-7):min(w,center+8)]
        strip=image[area]
        sharp=float(np.mean(sharpness[area]**2))
        clipped=float(np.mean((strip<6)|(strip>249)))
        pixels=.85*info.k[0]*.08*incidence/distance
        reasons=set()
        if abs(u-info.k[2])>.35*w:reasons.add('edge')
        if incidence<.65:reasons.add('oblique')
        if pixels<18:reasons.add('too_small')
        if clipped>.50:reasons.add('exposure')
        if sharp<18:reasons.add('low_detail_or_blur')
        yield points[i], normal, reasons


@dataclass
class Face:
    xy: np.ndarray
    normal: np.ndarray
    poor: int = 0
    good: int = 0
    queued: float | None = None
    observer: tuple = ()
    last_poor: float = 0.
    last_good: float = -100.
    confirmed: bool = False
    revisited: bool = False
    reasons: Counter = field(default_factory=Counter)


class SurfaceMemory:
    def __init__(self):
        self.faces=[]
        self.buckets=defaultdict(list)
        self.last_request=-100.
        self.requests={}
        self.recent_regions=[]

    def nearest(self, point, normal=None):
        x,y=np.floor(point/.25).astype(int)
        best,limit=None,.20**2
        for i in range(x-1,x+2):
            for j in range(y-1,y+2):
                for idx in self.buckets.get((i,j),()):
                    f=self.faces[idx]
                    if normal is not None and f.normal@normal<.82:continue
                    d=f.xy-point
                    dist=float(d@d)
                    if dist<limit:best,limit=idx,dist
        return best

    def update(self, observations, observer, now):
        per_frame={}
        for point,normal,reasons in observations:
            idx=self.nearest(point,normal)
            if idx is None:
                idx=len(self.faces)
                self.faces.append(Face(point.copy(),normal.copy()))
                self.buckets[tuple(np.floor(point/.25).astype(int))].append(idx)
            if idx not in per_frame or len(reasons)<len(per_frame[idx]):
                per_frame[idx]=reasons
        for idx,reasons in per_frame.items():
            f=self.faces[idx]
            if f.confirmed:continue
            if reasons:
                f.poor=f.poor+1 if now-f.last_poor<=1.5 else 1
                f.last_poor=now
                f.reasons.update(reasons)
                if f.poor>=2 and f.queued is None:
                    f.queued=now
                    f.observer=tuple(observer)
            else:
                f.good=f.good+1 if now-f.last_good<=1.5 else 1
                f.last_good=now
                if f.good>=2:
                    f.confirmed=True
                    if f.queued is not None:
                        f.revisited=now-f.queued>=3 and math.dist(observer[:2],f.observer[:2])>=.30

    def pending(self, now):
        return [i for i,f in enumerate(self.faces) if f.queued is not None and not f.confirmed
                and now-f.last_poor<=120 and now-f.queued>=2 and now-self.requests.get(i,-1e20)>=30]

    def choose(self, snapshot, pose, now, focal, mapped_ids):
        """Rerank only saved feasible alternatives; do not override marker-directed decisions."""
        from osp_mission.edge_policy import visible
        rows=snapshot['candidates']
        base=int(snapshot['selected_index'])
        if base<0 or len(rows)<2 or now-self.last_request<45:
            return None
        # Avoid inspecting neighboring samples of the same side on successive decisions.
        self.recent_regions=[(k,t) for k,t in self.recent_regions if now-t<90]
        ids=[i for i in mapped_ids if all(
            np.linalg.norm(self.faces[i].xy-self.faces[k].xy)>.8 or
            self.faces[i].normal@self.faces[k].normal<.82 for k,_ in self.recent_regions)]
        if not ids:return None
        xy=np.stack([self.faces[i].xy for i in ids])
        normals=np.stack([self.faces[i].normal for i in ids])
        delta=pose[2]-float(snapshot['heading'])
        c,s=math.cos(delta),math.sin(delta)
        rotation=np.array([[c,-s],[s,c]])
        origin=snapshot['origin'];res=float(snapshot['resolution']);data=snapshot['data']
        robot=snapshot['robot']
        wall_map=(xy-pose[:2])@rotation+robot
        wall_grid=(wall_map-origin)/res
        def views(i):
            row=rows[i]
            pos=(row[:2]-robot)@rotation.T+pose[:2]
            v=pos-xy
            dist=np.linalg.norm(v,axis=1)
            inc=(v*normals).sum(axis=1)/np.maximum(.001,dist)
            # Need an observed face and a clear look from a known reachable candidate.
            candidates=np.flatnonzero((inc>=.65)&(dist>=.40)&(dist<=.85*focal*.08*inc/18))
            if not len(candidates):return []
            cell=(row[:2]-origin)/res
            # Stop before the obstacle: the physical anchor may be inside the occupied cell.
            ray=wall_grid[candidates]-cell
            tip=wall_grid[candidates]-ray/np.maximum(1.,np.linalg.norm(ray,axis=1))[:,None]*1.5
            clear=visible(data,cell[1],cell[0],tip[:,1],tip[:,0])
            return [ids[k] for k in candidates[clear]]
        # A baseline destination that already affords a second look keeps priority.
        base_faces=views(base)
        def already_covered(fid):
            delta_xy=wall_map[ids.index(fid)]-rows[base,:2]
            bearing=math.atan2(delta_xy[1],delta_xy[0])
            yaw, sweep=rows[base,2:4]
            center=yaw+sweep/2
            offset=math.atan2(math.sin(bearing-center), math.cos(bearing-center))
            # Require the bearing to cross the central portion of the saved camera sweep.
            return abs(offset)<=abs(sweep)/2+math.radians(10)
        if any(already_covered(fid) for fid in base_faces):return None
        choices=[base] if base_faces else sorted((i for i in range(len(rows)) if i!=base and rows[i,6]<=rows[base,6]+1.),
                       key=lambda i:rows[i,6])
        for i in choices:
            visible_ids=views(i)
            if not visible_ids:continue
            fid=max(visible_ids,key=lambda k:self.faces[k].last_poor)
            target=wall_map[ids.index(fid)]
            aim=math.atan2(target[1]-rows[i,1],target[0]-rows[i,0])
            self.requests[fid]=now;self.last_request=now
            self.recent_regions.append((fid,now))
            return dict(kind="turn" if i==base else "waypoint", baseline_xy=rows[base,:2].tolist(),goal_xy=rows[i,:2].tolist(),
                        goal_yaw=aim,face_id=fid,extra_path_m=float(rows[i,6]-rows[base,6]))
        return None
