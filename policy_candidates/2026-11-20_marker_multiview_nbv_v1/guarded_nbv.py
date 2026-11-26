"""Cost-bounded local NBV. Preserve the adaptive scheduler's destination and phase."""
import math
import numpy as np
from osp_mission import _r32_navigation_core as r32
from osp_mission.adaptive_faces import clearance_grid
from osp_mission.center_wall import CenterWall
from osp_mission.marker_nbv import MarkerNBV
from osp_mission.nbv_faces import MultiView,angle,axis,wrap

class LocalMultiView(MultiView):
    def __init__(self):
        super().__init__()
        self.last_extra=-math.inf

    @staticmethod
    def independent(gains,nearby):
        # Adjacent, collinear map bins can describe a single physical side.
        result={}
        for f in sorted(nearby,key=lambda f:-gains.get(f[0],0.)):
            k,p,n,_,_,_=f
            if k not in gains:
                continue
            if any(n@other[2]>.94 and abs((p-other[1])@n)<.14 and math.dist(p,other[1])<1.6
                   for other in result.values()):
                continue
            result[k]=f
        return {k:gains[k] for k in result}

    def decide(self,grid,cost,pose,transform,viewed,elapsed,duration,veto,choices=(),fitted=(),probe_only=False):
        base=CenterWall.decide(self,grid,cost,pose,transform,viewed,elapsed,duration,veto,
                               choices,fitted,probe_only)
        if probe_only or self.camera is None or not base or not base.get('face_inspection'):
            return base
        if not {'surface_map','surface_odom','normal_odom'} <= base.keys():
            # A Flow frontier can carry a post-arrival scan without a geometric face.
            # Delegate to its caller's original Flow selection, exactly as the mission
            # does when an optional Hybrid proposal is unavailable.
            return None
        local=self.geometry(grid,transform,pose,fitted)
        focal,fov,ex,ey,ea=self.camera
        robot=np.asarray(pose[:2])
        goal=np.array([base['x'],base['y']])
        face=np.asarray(base['surface_map'])
        normal=axis(base['normal_odom']-transform[2])
        key=self.register(np.asarray(base['surface_odom']),base['normal_odom'])
        primary=next((f for f in local if f[0]==key),
                     (key,face,normal,.24,self.phase,False))
        nearby=[f for f in local if f[0]!=key and math.dist(f[1],goal)<3.45]
        nearby.append(primary)
        def visibility(position,heading):
            camera=position+ex*axis(heading)+ey*axis(heading+math.pi/2)
            q={}
            for f in nearby:
                q[f[0]]=self.quality(grid,camera,heading+ea,focal,fov,f)
            gains={k:v-self.faces[k].best for k,v in q.items() if v-self.faces[k].best>=.13}
            return q[key],self.independent(gains,nearby)
        sweep=base.get('scan_angle',0.)
        nominal=base['yaw']+sweep*.5
        primary_quality,gains=visibility(goal,nominal)
        safe,walk=grid.reachable(robot,.32)
        nav=r32.connected_nav_cells(cost,pose)
        clearance=clearance_grid(grid)
        def legal(point):
            a=grid.index(*point)
            b=cost.index(*point)
            return (grid.inside(*a) and cost.inside(*b) and safe[a] and np.isfinite(walk[a])
                    and clearance[a]>=.32 and nav[b])
        if not legal(goal) or not legal(robot):
            return base
        base_walk=walk[grid.index(*goal)]
        best=None
        best_gain=sum(gains.values())+.15
        tangent=np.array([-normal[1],normal[0]])
        for dn,dt in ((0.,0.),(.19,0.),(-.15,0.),(0.,.22),(0.,-.22),(.12,.30),(.12,-.30)):
            position=goal+dn*normal+dt*tangent
            if not legal(position):
                continue
            travel=walk[grid.index(*position)]
            if travel>base_walk+.25 or math.dist(position,face)>1.5 or (position-face)@normal<=0.:
                continue
            if not grid.line_clear(position,face+.09*normal):
                continue
            for delta in (0.,-.18,.18):
                heading=angle(face-position)-ea+delta
                q,gain=visibility(position,heading)
                if q<max(.24,primary_quality-.035) or len(gain)<2:
                    continue
                utility=(sum(gain.values())-.40*max(0.,travel-base_walk)
                         -.15*abs(wrap(heading-nominal))-.20*math.dist(position,goal))
                if utility<=best_gain:
                    continue
                g=dict(base,x=float(position[0]),y=float(position[1]),yaw=float(heading-sweep/2),
                       distance=math.dist(robot,position),nbv_face=key,nbv_gain=gain,
                       nbv_score=float(utility),clearance_m=float(clearance[grid.index(*position)]),
                       baseline_offset_m=math.dist(position,goal))
                if not veto(g):
                    best_gain,best=utility,g
        if best:
            self.reason='same_destination_multiview'
            return best
        # An already weakly observed side can get a short turn, never a new long detour.
        if elapsed-self.last_extra<45. or duration-elapsed<60.:
            return base
        for f in nearby:
            k,p,n,_,_,_=f
            old=self.faces[k]
            stand=math.dist(robot,p)
            if not (.15<old.best<.68 and .4<stand<=1.4 and old.visits<2 and elapsed-old.last>=45.):
                continue
            heading=angle(p-robot)-ea
            turn=abs(wrap(heading-pose[2]))
            camera=robot+ex*axis(pose[2])+ey*axis(pose[2]+math.pi/2)
            q=self.quality(grid,camera,heading+ea,focal,fov,f)
            if .12<turn<.45 and q>old.best+.25:
                g=dict(x=float(robot[0]),y=float(robot[1]),yaw=float(heading-.08),scan_angle=.08,
                       stationary=True,face_inspection=True,features=np.zeros(5),distance=0.,
                       r32_kind='quality_relook',nbv_face=k,nbv_gain={k:q-old.best},nbv_score=q-old.best,
                       normal_odom=old.normal,surface_odom=old.point.tolist(),surface_map=p.tolist())
                if not veto(g):
                    return g
        return base

    def dispatched(self,goal,elapsed):
        if goal.get('r32_kind')=='quality_relook':
            self.last_extra=elapsed
        super().dispatched(goal,elapsed)

class GuardedNBV(MarkerNBV):
    def __init__(self):
        super().__init__()
        self.schedule=LocalMultiView()
