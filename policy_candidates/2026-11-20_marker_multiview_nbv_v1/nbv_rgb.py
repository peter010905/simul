"""Anonymous evidence only. Bearings can authorize a look; moving needs measured range."""
from dataclasses import dataclass
import math
import numpy as np
from osp_mission import _r32_navigation_core as r32
from osp_mission.r32_drive import in_map
from osp_mission.nbv_faces import angle, axis, wrap, clearance_grid

@dataclass
class Track:
    id: int
    stamp: float
    origin: np.ndarray
    bearing: float
    point: object = None
    uv: tuple = ()
    distance: object = None
    quality: float = 0.
    confidence: float = 0.
    count: int = 0
    looks: int = 0
    approaches: int = 0
    used_at: float = -math.inf
    lost: bool = False
    resolved: bool = False

class RGBMemory:
    def __init__(self):
        self.tracks = []
        self.used = []  # base dispatch telemetry expects this list
        self.telemetry = dict(observations=0,bearing_only=0,found=0,strengthened=0,lost=0,registered=0)
        self.sequence = 0
        self.revision = 0
        self.event = 'start'

    def changed(self,why):
        self.telemetry[why] += 1
        self.event = why
        self.revision += 1

    def expire(self,now):
        for t in self.tracks:
            if not t.lost and now-t.stamp>3.:
                t.lost=True
                self.changed('lost')
        self.tracks = [t for t in self.tracks if now-t.stamp<=90.]

    def observe(self,e,now):
        stamp = float(e['stamp'])
        origin = np.asarray(e['observer'],float)
        uv = tuple(map(float,e['uv']))
        bearing = float(e['bearing'])
        size = float(e['size'])
        if (origin.shape!=(2,) or len(uv)!=2 or not np.isfinite([*origin,*uv,bearing,size,stamp]).all()
                or not -.1<=now-stamp<=.65 or size<=0):
            return
        p = np.asarray(e['point'],float) if e.get('point') is not None else None
        dist = None if p is None else math.dist(p,origin)
        if p is not None and (p.shape!=(2,) or not np.isfinite(p).all() or not .25<dist<8.):
            return
        self.telemetry['observations'] += 1
        self.telemetry['bearing_only'] += int(p is None)
        self.expire(now)
        def error(t):
            if p is not None and t.point is not None:
                return math.dist(p,t.point)/min(.30,.11+.035*dist)
            if now-t.stamp>1.4 or math.dist(t.origin,origin)>.22:
                return math.inf
            expected = angle(t.point-origin) if t.point is not None else t.bearing
            return abs(wrap(expected-bearing))/.058
        nearest = min(self.tracks,key=error,default=None)
        new = nearest is None or error(nearest)>1.
        if new:
            self.sequence+=1
            nearest = Track(self.sequence,stamp,origin,bearing)
            self.tracks.append(nearest)
            self.tracks = self.tracks[-64:]
        elif nearest.resolved or stamp-nearest.stamp<.075:
            return
        t=nearest
        previous_confidence = t.confidence
        gap = max(0.,stamp-t.stamp)
        if p is not None:
            t.point = p if t.point is None else .45*p+.55*t.point
        t.origin,t.bearing,t.uv,t.stamp,t.lost = origin,bearing,uv,stamp,False
        t.distance = None if t.point is None else math.dist(t.point,origin)
        edge = min(1.,max(0.,min(uv[0],1-uv[0],uv[1],1-uv[1]))/.16)
        t.quality = min(1.,size/24.)*(.64+.36*edge)
        evidence = (.26+.29*t.quality)*(1. if p is not None else .75)
        t.confidence = 1.-(1.-previous_confidence*math.exp(-gap/4.))*(1.-evidence)
        t.count+=1
        if new or gap>3. or int(previous_confidence*4)<int(t.confidence*4):
            self.changed('found' if new or gap>3. else 'strengthened')

    def reconcile(self,targets,transform):
        for t in self.tracks:
            if t.resolved or t.point is None:
                continue
            point = in_map(('odom',*t.point),transform)
            if any(math.dist(point,target[:2])<.26 for target in targets.values()):
                t.resolved=True
                self.changed('registered')

    def choose(self,grid,cost,pose,transform,targets,now,veto):
        if transform is None:
            return None
        self.expire(now)
        self.reconcile(targets,transform)
        recent = [t for t in self.tracks if not t.resolved and not t.lost and now-t.used_at>=2.]
        if not recent:
            return None
        robot = np.asarray(pose[:2])
        safe,walk = grid.reachable(robot,.32)
        nav = r32.connected_nav_cells(cost,pose)
        clear = clearance_grid(grid)
        def legal(pos):
            cell,cc=grid.index(*pos),cost.index(*pos)
            return (grid.inside(*cell) and cost.inside(*cc) and nav[cc] and safe[cell]
                    and np.isfinite(walk[cell]) and clear[cell]>=.32)
        if not legal(robot):
            return None
        winner = None
        for t in recent:
            observer=np.asarray(in_map(('odom',*t.origin),transform))
            bearing=t.bearing-transform[2]
            # With no range, retain only the direction and only near the actual observer.
            if t.point is None and math.dist(observer,robot)>.20:
                continue
            point = np.asarray(in_map(('odom',*t.point),transform)) if t.point is not None else observer+2.*axis(bearing)
            if math.dist(point,robot)<.10:
                continue
            heading = angle(point-robot)
            turn = abs(wrap(heading-pose[2]))
            live = t.confidence*math.exp(-max(0.,now-t.stamp)/6.)
            actions = []
            if t.looks<1 and turn<1.55 and (t.point is None or grid.line_clear(robot,point+.12*axis(heading+math.pi))):
                # Centering an edge detection is cheaper than driving to it.
                benefit=live*(.45+.65*min(1.,turn/.5)+.28*(1.-t.quality))
                actions.append((benefit/(1.+.45*turn),robot,'relook'))
            if (t.point is not None and t.count>=3 and live>=.62 and not t.approaches
                    and math.dist(robot,point)>1.1):
                normal = (observer-point)/max(.01,math.dist(observer,point))
                tangent = np.array([-normal[1],normal[0]])
                for stand in (.85,1.15,1.4):
                    for shift in (0.,-.3,.3):
                        pos = point+stand*normal+shift*tangent
                        if not legal(pos) or math.dist(pos,robot)<.25 or not grid.line_clear(pos,point+.12*normal):
                            continue
                        travel = walk[grid.index(*pos)]
                        gain = live*(1.1+.45*(1.-t.quality))/(1.+.30*travel+.14*turn)
                        actions.append((gain,pos,'approach'))
            for score,pos,action in actions:
                if score<.25 or (winner and score<=winner[0]):
                    continue
                goal=dict(x=float(pos[0]),y=float(pos[1]),yaw=angle(point-pos),distance=math.dist(pos,robot),
                          scan_angle=0.,features=np.zeros(5),stationary=action=='relook',print_point=list(point),
                          nbv_rgb=t.id,nbv_action=action,nbv_score=float(score),cue_quality=t.quality,
                          confidence=t.confidence,uv=t.uv,observations=t.count,last_seen=t.stamp,
                          range_m=t.distance,measured_odom=t.point.tolist() if t.point is not None else None)
                if not veto(goal):
                    winner=score,goal
        return winner[1] if winner else None

    def dispatched(self,goal,stamp):
        for t in self.tracks:
            if t.id==goal.get('nbv_rgb'):
                t.used_at=stamp
                if goal['nbv_action']=='approach':
                    t.approaches+=1
                else:
                    t.looks+=1
