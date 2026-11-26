"""Hold weak bearings during box inspection; rank only already-feasible wall viewpoints."""
from collections import deque
from dataclasses import dataclass, field
import math
import numpy as np
from osp_mission.r32_drive import in_map


def wrap(a):
    return math.atan2(math.sin(a),math.cos(a))


@dataclass
class Bearing:
    samples: deque = field(default_factory=lambda: deque(maxlen=12))
    last: float = -math.inf
    used: bool = False

    def stable(self):
        return (len(self.samples)>=3 and self.samples[-1][0]-self.samples[0][0]>=.25)


class WallBearings:
    def __init__(self):
        self.tracks = []
        self.tried = []

    def observe(self, stamp, now, origin, bearing, size):
        if (not all(map(math.isfinite, (*origin,bearing,stamp,now,size))) or
                not -.1 <= now-stamp <= .6 or not 6 <= size < 10):
            return
        self.tracks = [t for t in self.tracks if now-t.last <= (600 if t.stable() else 1.5)]
        options = []
        for t in self.tracks:
            _,p,a,s=t.samples[-1]
            if (0 <= stamp-t.last <= .8 and math.dist(origin,p)<=.45 and
                    abs(wrap(bearing-a)) <= math.radians(2.2) and .64<=size/s<=1.56):
                options.append((abs(wrap(bearing-a)),t))
        if options:
            t=min(options,key=lambda v:v[0])[1]
            if stamp-t.last<.075:
                return
        else:
            if len(self.tracks)>=96:
                self.tracks.remove(min(self.tracks,key=lambda t:(t.stable() and not t.used,t.last)))
            t=Bearing()
            self.tracks.append(t)
        t.samples.append((stamp,tuple(origin),bearing,size))
        t.last=stamp

    def choose(self, walls, grid, transform, known, now, time_left):
        if not walls or time_left<8 or len(self.tried)>=2:
            return None
        usable=[t for t in self.tracks if t.stable() and not t.used and 0<=now-t.last<=600]
        if not usable:
            return None
        anchors=np.asarray([g['surface_odom'] for _,g in walls])
        # Only untried locations. Map geometry and original clearance checks still decide where to stand.
        allowed=np.ones(len(walls),dtype=bool)
        for p in known:
            allowed &= np.linalg.norm(anchors-p,axis=1)>.35
        for p in self.tried:
            allowed &= np.linalg.norm(anchors-p,axis=1)>.8
        votes = {}
        for track in usable:
            ids=allowed.copy()
            error=np.zeros(len(walls))
            rays=[track.samples[0],track.samples[len(track.samples)//2],track.samples[-1]]
            for _,origin,angle,_ in rays:
                delta=anchors-origin
                length=np.linalg.norm(delta,axis=1)
                axis=np.array([math.cos(angle),math.sin(angle)])
                lateral=np.abs(delta[:,0]*axis[1]-delta[:,1]*axis[0])
                ids &= (delta@axis>0)&(length>=1.3)&(length<=25)&(lateral<.22+.018*length)
                error+=lateral/(.22+.018*length)
            matches=[]
            for i in np.flatnonzero(ids):
                cost,g=walls[i]
                if cost/.22+6>time_left:
                    continue
                surface=np.array(g['surface_map'])
                camera=np.asarray(in_map(('odom',*rays[-1][1]),transform))
                vector=camera-surface
                norm=np.linalg.norm(vector)
                if norm<.1 or vector@(np.array([g['x'],g['y']])-surface)<=0:
                    continue
                if grid.line_clear(camera,surface+.12*vector/norm):
                    matches.append((error[i],int(i),norm))
            if not matches:
                continue
            _,_,depth=min(matches)
            if any(abs(depth-d)>1.2 for _,_,d in matches):
                continue
            for residual,i,_ in matches:
                votes.setdefault(i,[]).append((track,float(residual/3)))
        best = None
        for i, supports in votes.items():
            tracks = [track for track,_ in supports]
            stronger = any(sum(size >= 8. for _,_,_,size in t.samples) >= 2 for t in tracks)
            # Two glimpses from distinct positions can corroborate a very small silhouette.
            origins = [t.samples[0][1] for t in tracks]
            independent = any(math.dist(a,b)>=.50 for j,a in enumerate(origins) for b in origins[j+1:])
            if not stronger and not independent:
                continue
            cost, goal = walls[i]
            score = cost + min(e for _,e in supports) + (0. if stronger else 1.)
            if best is None or score < best[0]:
                best = (score,goal,[(t.samples[0][0],t.samples[0][2]) for t in tracks])
        if best is None:
            return None
        return dict(best[1], r32_kind='wall_surface',weak_wall=True,weak_keys=best[2])

    def dispatched(self,goal):
        if goal.get('weak_wall'):
            self.tried.append(tuple(goal['surface_odom']))
            for track in self.tracks:
                if (track.samples[0][0], track.samples[0][2]) in goal['weak_keys']:
                    track.used = True
