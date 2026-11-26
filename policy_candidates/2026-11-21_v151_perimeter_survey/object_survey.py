"""Finish camera-confirmed object faces, defer inaccessible sides, visit new areas."""
from collections import deque
import hashlib
import math
import numpy as np
from osp_mission.adaptive_faces import inspection_pose
from osp_mission.box_views import odom_point
from osp_mission.edge_policy import DIRECTIONS
from osp_mission.face_tasks import FaceTasks,face_evidence,local_face
from osp_mission.perimeter_route import PerimeterRoute


class ObjectSurvey:
    def __init__(self):
        self.tasks=FaceTasks()
        self.visits=deque(maxlen=1800)
        self.active_box=None
        self.active_since=0.
        self.active_face=None
        self.arrived_face=None
        self.region=None
        self.rejections={}
        self.finished=set()
        self.last_reason='start'
        self.perimeter=PerimeterRoute()
        self.object_budget=90.

    def observe_pose(self,pose,transform):
        p=odom_point(pose[:2],transform)
        if not self.visits or np.linalg.norm(p-self.visits[-1])>=.25:
            self.visits.append(p)
        if self.region is not None and np.linalg.norm(p-self.region)<.65:self.region=None
        return p

    @staticmethod
    def signature(grid,cost,point,nav):
        digest=hashlib.blake2s(digest_size=12)
        for raster,array in ((grid,grid.cells),(cost,cost.cells),(cost,nav)):
            row,col=raster.index(*point);span=math.ceil(1.6/raster.resolution)
            crop=array[max(0,row-span):row+span+1,max(0,col-span):col+span+1]
            if array.dtype!=bool:crop=np.digitize(crop,[0,26,65,80,99])
            digest.update(str(crop.shape).encode());digest.update(crop.astype(np.uint8).tobytes())
        return digest.digest()

    def owner(self,goal,transform):
        point=goal.get('surface_odom')
        if point is None:return None
        p=np.asarray(point)
        found=[]
        for box in self.tasks.boxes:
            for f in box.faces:
                delta=p-f.point;tangent=np.array([-f.normal[1],f.normal[0]])
                if abs(delta@f.normal)<.16 and abs(delta@tangent)<=f.half+.10:
                    found.append(box.ident)
        return found[0] if found and len(set(found))==1 else None

    def update(self,samples,grid,viewed,transform,fitted,elapsed):
        self.tasks.fit(fitted,transform,elapsed)
        groups=[(side,r,c) for (_,_,side,_,_),cells in samples.items() for r,c in cells]
        if groups:
            side,row,col=np.array(groups,int).T
            dy,dx=DIRECTIONS[side].T
            xy=np.array([grid.world(r+.5*d,c+.5*e) for r,c,d,e in zip(row,col,dy,dx)])
            theta=grid.origin[2];c,s=math.cos(theta),math.sin(theta)
            normals=np.column_stack((dx*c-dy*s,dx*s+dy*c))
            seen=viewed[side,row,col]
            for face in self.tasks.by_key.values():
                p,n=local_face(face,transform)
                face_evidence(face,p,n,xy,normals,seen,grid.resolution,.70)
        self.perimeter.observe(samples,grid,viewed,transform,self.owner,elapsed)
        self.finished={b.ident for b in self.tasks.boxes if all(f.state=='OBSERVED' for f in b.faces)}
        if self.arrived_face:
            f=self.tasks.by_key[self.arrived_face]
            if f.state!='OBSERVED':
                f.fail('post_sweep_camera_evidence_insufficient',elapsed)
            self.arrived_face=None

    def choose(self,grid,cost,pose,transform,viewed,elapsed,duration,veto,choices,samples,
               candidates,distances,clearance,nav,fitted):
        self.observe_pose(pose,transform)
        self.update(samples,grid,viewed,transform,fitted,elapsed)
        views=[]
        for box in self.tasks.boxes:
            if box.ident in self.finished:continue
            p,_=local_face(box.faces[0],transform)
            revision=self.signature(grid,cost,p,nav)
            for f in box.faces:
                if not f.eligible(revision,elapsed):continue
                f.probe(revision,elapsed)
                p,n=local_face(f,transform)
                fields=dict(object_face=f.key,object_id=box.ident,r32_kind='interior_surface',
                    surface_map=p.tolist(),surface_odom=f.point.tolist(),normal_odom=math.atan2(f.normal[1],f.normal[0]),
                    features=np.zeros(5),previous_attempts=0,scan_angle=math.radians(36),face_inspection=True,stationary=False)
                view=inspection_pose(grid,cost,nav,distances,clearance,p,n,f.half,pose,fields,veto)
                if view:
                    if f.state in ('FAILED','BLOCKED'):f.state,f.reason='PENDING','accessible_again'
                    views.append(view)
                else:f.blocked(elapsed)
        # Retire generic proposals on every owned surface, including unseen but
        # currently inaccessible sides. Only the face task can reopen those.
        for kind in candidates:
            candidates[kind]=[(d,g) for d,g in candidates[kind] if self.owner(g,transform) is None]
        # A detour serves nearby objects. Remote deferred faces wait for a pass
        # near that object instead of pulling the robot back across the room.
        views=[(walk,g) for walk,g in views if g['object_id']==self.active_box or walk<=3.5]
        if views:
            if self.active_box is None or not any(g['object_id']==self.active_box for _,g in views):
                self.active_box=min(views,key=lambda x:x[0])[1]['object_id']
                self.active_since=elapsed
                spare=duration-elapsed-self.perimeter.remaining_estimate_s
                self.object_budget=min(90.,max(25.,spare/2.))
            local=[v for v in views if v[1]['object_id']==self.active_box]
            if elapsed-self.active_since<=self.object_budget:
                self.last_reason='finish_accessible_object_faces'
                return min(local,key=lambda x:x[0])[1]
            for _,g in local:
                f=self.tasks.by_key[g['object_face']]
                f.fail('object_inspection_time_budget',elapsed)
            views=[v for v in views if v[1]['object_id']!=self.active_box]
            self.active_box=None
        else:self.active_box=None
        perimeter=self.perimeter.choose(grid,cost,pose,transform,distances,clearance,nav,veto,elapsed,duration)
        if perimeter is not None:
            self.last_reason='resume_perimeter_after_object'
            return perimeter
        region=self.next_region(grid,cost,pose,transform,choices,distances,nav,veto,elapsed,duration)
        if region:
            self.last_reason='visit_camera_unvisited_region'
            return region
        if views:
            self.active_box=min(views,key=lambda x:x[0])[1]['object_id'];self.active_since=elapsed
            self.last_reason='next_object'
            return min(views,key=lambda x:x[0])[1]
        self.last_reason='remaining_walls_or_generic_surfaces'
        remaining=candidates['wall'] or candidates['interior']
        return min(remaining,key=lambda item:item[0])[1] if remaining else None

    def next_region(self,grid,cost,pose,transform,choices,distances,nav,veto,elapsed,duration):
        proposals=[]
        history=np.asarray(self.visits)
        for g in choices:
            if g.get('stationary') or any(k in g for k in ('approach_id','focus_id','print_point')):continue
            point=(g['x'],g['y']);cell=grid.index(*point);cc=cost.index(*point)
            if not grid.inside(*cell) or not cost.inside(*cc) or not nav[cc] or veto(g):continue
            walk=float(distances[cell])
            if not math.isfinite(walk) or walk/.20+8.>duration-elapsed:continue
            odom=odom_point(point,transform)
            novelty=np.linalg.norm(history-odom,axis=1).min()
            if novelty<1.8:continue
            if any(elapsed-t<60 and np.linalg.norm(odom-p)<1. for p,t in self.rejections.values()):continue
            commitment=self.region is not None and np.linalg.norm(odom-self.region)<.8
            reward=3.*commitment+min(5.,novelty)+.3*max(0.,float(g['features'][1]))
            proposals.append((reward/(1.+.06*walk),dict(g,region_transfer=True,region_odom=odom.tolist(),r32_kind='interior_frontier')))
        return max(proposals,key=lambda x:x[0])[1] if proposals else None

    def dispatched(self,goal,elapsed):
        self.perimeter.dispatched(goal)
        self.active_face=goal.get('object_face')
        if self.active_face:
            f=self.tasks.by_key[self.active_face];f.attempts+=1;f.last_dispatch=elapsed
        if goal.get('region_transfer'):self.region=np.array(goal['region_odom'])

    def outcome(self,status,reason,elapsed):
        self.perimeter.outcome(status,reason,elapsed)
        if self.active_face:
            f=self.tasks.by_key[self.active_face]
            if status==4:self.arrived_face=f.key
            elif status==6 or reason in ('navigation_stalled','action_timeout','bump_contact'):
                f.fail(reason or 'nav2_failed',elapsed)
            self.active_face=None
        if self.region is not None and (status==6 or reason in ('navigation_stalled','action_timeout','bump_contact')):
            self.rejections[tuple(self.region)]=(self.region.copy(),elapsed);self.region=None

    def report(self):
        return dict(perimeter=self.perimeter.report(),completed_objects=sorted(self.finished),active_object=self.active_box,
                    region=self.region.tolist() if self.region is not None else None,reason=self.last_reason,
                    objects={str(b.ident):[f.public() for f in b.faces] for b in self.tasks.boxes})
