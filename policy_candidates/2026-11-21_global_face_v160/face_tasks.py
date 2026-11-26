"""Long-lived object identities and independently retryable odom-frame faces."""
from dataclasses import dataclass, field
import math
import numpy as np
from scipy.optimize import linear_sum_assignment
from osp_mission.box_views import odom_point


@dataclass
class FaceTask:
    key: str
    point: np.ndarray
    normal: np.ndarray
    half: float
    since: float
    state: str = 'PENDING'
    reason: str = 'awaiting_view'
    fraction: float = 0.
    attempts: int = 0
    failures: int = 0
    probes: int = 0
    revision: object = None
    retry_at: float = 0.
    last_try: float = -1e6
    last_dispatch: float = -1e6

    def geometry_reset(self, now):
        self.state,self.reason,self.fraction='PENDING','fit_changed',0.
        self.revision=None
        self.failures=0
        self.retry_at=self.since=now

    def eligible(self, revision, now):
        if self.state=='OBSERVED': return False
        changed=revision!=self.revision
        if self.state=='BLOCKED':
            return (changed and now-self.last_try>=3.) or now>=self.retry_at
        if self.state=='FAILED':
            return (changed and now-self.last_try>=6.) or (self.failures<3 and now>=self.retry_at)
        return True

    def probe(self, revision, now):
        if revision!=self.revision:
            self.failures=0
        self.revision=revision
        self.last_try=now
        self.probes+=1

    def blocked(self,now):
        self.state,self.reason='BLOCKED','no_safe_visible_connected_view'
        self.retry_at=now+45.

    def fail(self,reason,now):
        self.state,self.reason='FAILED',reason
        self.failures+=1
        self.last_try=now
        self.retry_at=now+min(90.,10.*2**min(3,self.failures-1))

    def public(self):
        return dict(id=self.key,state=self.state,reason=self.reason,confirmed_fraction=round(self.fraction,3),
                    attempts=self.attempts,failures_this_revision=self.failures,feasibility_checks=self.probes,
                    anchor_odom=self.point.tolist(),normal_odom=self.normal.tolist(),width_m=2*self.half)


@dataclass
class Box:
    ident: int
    center: np.ndarray
    faces: list=field(default_factory=list)


class FaceTasks:
    def __init__(self):
        self.boxes=[]
        self.by_key={}

    def fit(self,fitted,transform,now):
        fits=[]
        for center,faces,_ in fitted:
            if len(faces)!=4: continue
            geometry=[]
            for p,n in faces:
                n=np.asarray(n,float)
                tangent=np.array([-n[1],n[0]])
                width=max(abs((q-center)@tangent) for q,_ in faces)
                angle=math.atan2(n[1],n[0])+transform[2]
                geometry.append((odom_point(p,transform),np.array([math.cos(angle),math.sin(angle)]),width))
            fits.append((odom_point(center,transform),geometry))
        pairs={}
        if self.boxes and fits:
            costs=np.array([[np.linalg.norm(c-box.center) for box in self.boxes] for c,_ in fits])
            for i,j in zip(*linear_sum_assignment(costs)):
                if costs[i,j]<.40: pairs[i]=self.boxes[j]
        for i,(center,geometry) in enumerate(fits):
            if i not in pairs:
                box=Box(len(self.boxes)+1,center.copy())
                for j,(p,n,w) in enumerate(geometry):
                    face=FaceTask(f'{box.ident}.{j+1}',p,n,w,now)
                    box.faces.append(face); self.by_key[face.key]=face
                self.boxes.append(box)
                continue
            box=pairs[i]
            costs=np.array([[1.-float(f.normal@n)+np.linalg.norm(f.point-p) for p,n,_ in geometry] for f in box.faces])
            for j,k in zip(*linear_sum_assignment(costs)):
                f=box.faces[j]; p,n,w=geometry[k]
                if np.linalg.norm(f.point-p)>.13 or f.normal@n<.94 or abs(f.half-w)>.10: f.geometry_reset(now)
                f.point,f.normal,f.half=p,n,w
            box.center=center.copy()


def local_face(face,transform):
    tx,ty,theta=transform
    c,s=math.cos(theta),math.sin(theta)
    inverse=np.array([[c,s],[-s,c]])
    return inverse@(face.point-[tx,ty]), inverse@face.normal


def face_evidence(face, point, normal, xy, normals, viewed, resolution, required):
    # Five width bins require support across the face, including as-yet unmapped parts.
    tangent=np.array([-normal[1],normal[0]])
    delta=xy-point
    along=delta@tangent
    ids=(abs(delta@normal)<=max(.075,1.6*resolution)) & (abs(along)<=face.half+resolution/2) & (normals@normal>.67)
    bins=np.clip(((along[ids]+face.half)/max(.001,2*face.half)*5).astype(int),0,4)
    hit=np.bincount(bins[viewed[ids]],minlength=5)
    face.fraction=float(np.minimum(hit*resolution,2*face.half/5).sum()/max(.001,2*face.half))
    if face.fraction>=required:
        face.state,face.reason='OBSERVED','camera_confirmed_surface'
    elif face.state=='OBSERVED':
        face.state,face.reason='PENDING','evidence_no_longer_aligned'
