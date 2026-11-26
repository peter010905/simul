"""One candidate pool; only independently processed imagery can retire a surface task."""
from collections import Counter, deque
import hashlib
import math
import numpy as np
from scipy.spatial import cKDTree

from osp_mission.adaptive_faces import inspection_pose
from osp_mission.center_wall import CenterWall
from osp_mission.edge_policy import DIRECTIONS, visible
from osp_mission.box_views import odom_point
from osp_mission.face_tasks import FaceTasks, face_evidence, local_face


def surface_cloud(samples,grid,viewed):
    groups=[(side,r,c,kind=='wall') for (kind,_,side,_,_),cells in samples.items() for r,c in cells]
    if not groups:
        return np.empty((0,2)),np.empty((0,2)),np.empty(0,bool),np.empty(0,bool)
    side,r,c,wall=np.array(groups,int).T
    dy,dx=DIRECTIONS[side].T
    c0,s0=math.cos(grid.origin[2]),math.sin(grid.origin[2])
    rotation=np.array([[c0,-s0],[s0,c0]])
    xy=np.column_stack((c+.5+.5*dx,r+.5+.5*dy))*grid.resolution
    return xy@rotation.T+grid.origin[:2],np.column_stack((dx,dy))@rotation.T,viewed[side,r,c],wall.astype(bool)


def revision(grid,cost,center,reachable,nav):
    # Changes away from a box matter when they alter the reachable side of its local ring.
    digest=hashlib.blake2s(digest_size=12)
    for raster,values in ((grid,grid.cells),(cost,cost.cells),(grid,reachable),(cost,nav)):
        r,c=raster.index(*center); w=math.ceil(1.8/raster.resolution)
        crop=values[max(0,r-w):max(0,r+w+1),max(0,c-w):max(0,c+w+1)]
        # Occupancy classes match the safe-goal generator, including unknown.
        if values.dtype!=bool:
            crop=np.digitize(crop, [0,26,65,80,99])
        digest.update(str(crop.shape).encode());digest.update(crop.astype(np.uint8).tobytes())
    return digest.digest()


def optical_plane(info,transform,heading,height):
    from scipy.spatial.transform import Rotation
    q=transform.rotation
    optical=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    relative=Rotation.from_euler('z',-heading).as_matrix()@optical
    return (relative,height-transform.translation.z,float(info.k[0]),float(info.k[4]),
            float(info.k[2]),float(info.k[5]),info.width,info.height)


def in_image(x,y,camera,plane):
    rotation,height,fx,fy,cx,cy,width,rows=plane
    dx=x-camera[0];dy=y-camera[1];c,s=math.cos(camera[2]),math.sin(camera[2])
    rays=np.column_stack((c*dx+s*dy,-s*dx+c*dy,np.full_like(dx,height)))@rotation
    depth=rays[:,2];z=np.maximum(depth,.001)
    u=fx*rays[:,0]/z+cx;v=fy*rays[:,1]/z+cy
    return (depth>0)&(u>=0)&(u<width)&(v>=0)&(v<rows)


def camera_gain(grid,xy,normals,unseen_wall,tree,goal,camera):
    focal,half_fov,size,pixels,offset,*plane=camera
    heading=goal['yaw']+min(.65,max(0.,goal.get('scan_angle',0.)))/2
    co,si=math.cos(heading),math.sin(heading)
    cx=goal['x']+co*offset[0]-si*offset[1];cy=goal['y']+si*offset[0]+co*offset[1]
    heading+=offset[2]
    radius=.85*focal*size/pixels
    ids=np.asarray(tree.query_ball_point((cx,cy),radius),dtype=int)
    if not len(ids):return 0.,0.,ids
    delta=xy[ids]-[cx,cy]; dist=np.linalg.norm(delta,axis=1)
    inc=-np.einsum('ij,ij->i',delta,normals[ids])/np.maximum(dist,1e-9)
    angle=np.arctan2(delta[:,1],delta[:,0])-heading
    mask=(dist>.25)&(inc>.60)&(dist<radius*inc)&(np.cos(angle)>math.cos(half_fov))
    if plane:mask &= in_image(xy[ids,0],xy[ids,1],(cx,cy,heading),plane[0])
    ids=ids[mask]
    if not len(ids):return 0.,0.,ids
    # Grid rays are measured up to the free side of each boundary.
    ox,oy,ga=grid.origin; c,s=math.cos(ga),math.sin(ga)
    target=xy[ids]+normals[ids]*.004
    dx,dy=(target-[ox,oy]).T
    y=(-s*dx+c*dy)/grid.resolution;x=(c*dx+s*dy)/grid.resolution
    sy=(-s*(cx-ox)+c*(cy-oy))/grid.resolution;sx=(c*(cx-ox)+s*(cy-oy))/grid.resolution
    clear=visible(grid.cells,sy,sx,y,x)
    wall=unseen_wall[ids[clear]]
    return float(wall.sum()*grid.resolution),float((~wall).sum()*grid.resolution),ids[clear]


class GlobalCoverage(CenterWall):
    def __init__(self):
        super().__init__()
        self.selection_errors=0
        self.tasks=FaceTasks()
        self.camera=None
        self.threshold=.70
        self.ranked=[]
        self.waiting=False
        self.last_service=dict(wall=0.,interior=0.,frontier=0.)
        self.history_goals=deque(maxlen=64)
        self.active_face=None
        self.batch_id=None
        self.batch_since=0.
        self.batch_count=0
        self.nav_failures=Counter()
        self.surfaces={}
        self.frontier_count=None
        self.idle_from=None
        self.termination=None
        self.paths=None

    def attempts(self,kind,anchor,normal_yaw,radius):
        # A rolling window permits later use after geometry or access improves.
        return [t for t in super().attempts(kind,anchor,normal_yaw,radius) if self.elapsed-t<45.]

    def decide(self,grid,cost,pose,transform,viewed,elapsed,duration,veto,choices=(),fitted=(),probe_only=False):
        self.elapsed=elapsed
        if probe_only:return None
        self.tasks.fit(fitted,transform,elapsed)
        return super().decide(grid,cost,pose,transform,viewed,elapsed,duration,veto,choices,(),False)

    def collect(self,grid,cost,pose,transform,viewed,elapsed,duration,veto,choices,samples,
                candidates,distances,clearance,nav):
        xy,normals,observed,is_wall=surface_cloud(samples,grid,viewed)
        self.surfaces={k:dict(observed_cells=int(np.sum(observed&mask)),mapped_cells=int(mask.sum()))
                       for k,mask in [('wall',is_wall),('obstacle',~is_wall)]}
        pool=[]
        for family in ('interior','wall'):
            for _,g in candidates[family]:
                pool.append(dict(g,family=family))
        weak=self.weak.choose(candidates['wall'],grid,transform,self.weak_known,self.weak_clock,duration-elapsed)
        if weak is not None:pool.append(dict(weak,family='wall'))
        for g in choices:
            if g['features'][1]>.015 and not g.get('stationary') and not any(k in g for k in ('approach_id','focus_id')):
                pool.append(dict(g,family='frontier',r32_kind='interior_frontier'))
        self.frontier_count=sum(g['family']=='frontier' for g in pool)
        accessible=np.isfinite(distances)
        for box in self.tasks.boxes:
            center,_=local_face(box.faces[0],transform)
            sig=revision(grid,cost,center,accessible,nav)
            for face in box.faces:
                p,n=local_face(face,transform)
                face_evidence(face,p,n,xy,normals,observed,grid.resolution,self.threshold)
                if not face.eligible(sig,elapsed):continue
                face.probe(sig,elapsed)
                fields=dict(object_face=face.key,box_id=box.ident,r32_kind='interior_surface',family='interior',
                    face_inspection=True,scan_angle=math.radians(30),stationary=False,features=np.zeros(5),
                    surface_map=p.tolist(),surface_odom=face.point.tolist(),normal_odom=math.atan2(*face.normal[::-1]))
                found=False
                for offset in (0.,-.65,.65):
                    tangent=np.array([-n[1],n[0]])
                    view=inspection_pose(grid,cost,nav,distances,clearance,p+tangent*face.half*offset,
                                         n,face.half,pose,fields,veto,visibility_offset=.06)
                    if view:
                        _,g=view
                        if not grid.line_clear(np.array([g['x'],g['y']]),p+.06*n):continue
                        g['yaw']=math.atan2(p[1]-g['y'],p[0]-g['x'])-g['scan_angle']/2
                        pool.append(g);found=True
                if not found:face.blocked(elapsed)
                elif face.state in ('FAILED','BLOCKED'):face.state,face.reason='PENDING','accessible_again'
        self.ranked=self.rank(pool,grid,cost,pose,transform,elapsed,distances,clearance,nav,
                              xy[~observed],normals[~observed],is_wall[~observed],veto)
        self.counts=dict(interior=len(candidates['interior']),wall=len(candidates['wall']),
                         frontier=self.frontier_count,ranked=len(self.ranked))
        if self.ranked:self.idle_from=None
        elif self.idle_from is None:self.idle_from=elapsed
        return self.ranked[0] if self.ranked else None

    def rank(self,pool,grid,cost,pose,transform,elapsed,distances,clearance,nav,xy,normals,wall,veto):
        if not self.camera:return []
        tree=cKDTree(xy)
        unique={}
        for g in pool:
            point=(g['x'],g['y'])
            cell=grid.index(*point);nc=cost.index(*point)
            if not grid.inside(*cell) or not cost.inside(*nc) or clearance[cell]<.32 or not nav[nc] or veto(g):continue
            walk=float(distances[cell])
            if not math.isfinite(walk):continue
            key=(g.get('object_face'),g['family'],round(point[0]/.18),round(point[1]/.18),round(g['yaw']/.28))
            if key in unique and not g.get('weak_wall'):continue
            if self.paths and self.paths.rejected.get(self.paths.key(g),0)>self.paths.node.now():continue
            unique[key]=dict(g,grid_walk_m=walk)
        membership=np.full(len(xy),-1,dtype=int)
        live=[f for f in self.tasks.by_key.values() if f.state!='OBSERVED']
        for j,face in enumerate(live):
            p,n=local_face(face,transform)
            delta=xy-p
            match=(abs(delta@n)<max(.075,1.6*grid.resolution)) & (abs(delta@np.array([-n[1],n[0]]))<=face.half) & (normals@n>.67) & ~wall
            membership[match]=j
        goals=[]
        for g in unique.values():
            w,o,ids=camera_gain(grid,xy,normals,wall,tree,g,self.camera)
            frontier=max(0.,float(g['features'][1]))
            f=self.tasks.by_key.get(g.get('object_face'))
            if w+o<.045 and frontier<=.015 and not g.get('weak_wall'):continue
            # Saturate each surface class independently so long walls cannot dominate by area.
            reward=(1.-math.exp(-w/.70))+(1.-math.exp(-o/.55))+.55*min(1.,frontier)
            face_ids=membership[ids]
            hits=np.bincount(face_ids[face_ids>=0],minlength=len(live))
            covered=[f.key for f,count in zip(live,hits) if count*grid.resolution>=min(.16,.6*f.half)]
            reward+=.24*min(2,len(covered))
            g['predicted_object_faces']=covered
            family=g['family']
            age=elapsed-self.last_service[family]
            reward+=min(.75,age/120.)*min(1.,w+o+frontier)
            if f is not None and o>.08:
                reward+=.5*(1.-f.fraction)+min(.7,max(0.,elapsed-max(f.since,f.last_dispatch))/110.)
                if g['box_id']==self.batch_id and self.batch_count<3 and elapsed-self.batch_since<40.:
                    reward+=.45
            if g.get('weak_wall'):reward+=.65
            if not g.get('object_face') and not g.get('weak_wall') and 'surface_odom' in g:
                # Generic surface bins can duplicate a face already retired by camera evidence.
                if any(f.state=='OBSERVED' and math.dist(f.point,g['surface_odom'])<.28 and
                    f.normal@np.array([math.cos(g['normal_odom']),math.sin(g['normal_odom'])])>.8 for f in self.tasks.by_key.values()):
                    continue
            odom=odom_point((g['x'],g['y']),transform)
            repeats=sum(elapsed-t<100 and math.dist(odom,p)<.75 for p,t in self.history_goals)
            turn=abs(math.remainder(g['yaw']-pose[2],2*math.pi))
            g.update(wall_gain_m=w,object_gain_m=o,reward=reward,other_cost=.22*turn+.45*repeats,
                     coverage_odom=odom.tolist(),fair_age=age)
            g['nbv_score']=self.utility(g,g['grid_walk_m'])
            goals.append(g)
        goals.sort(key=lambda g:-g['nbv_score'])
        # Allocate one real-planner shortlist slot to an overdue, still useful family.
        # Only vetted, nonzero-gain candidates can receive a turn.
        if not goals:return []
        eligible=[]
        for g in goals:
            f=self.tasks.by_key.get(g.get('object_face'))
            current=g.get('box_id')==self.batch_id and self.batch_id is not None
            if current and (self.batch_count>=3 or elapsed-self.batch_since>=40.) and any(
                    h.get('box_id')!=self.batch_id for h in goals):
                continue
            age=g['fair_age']
            if f is not None and f.key in g['predicted_object_faces']:
                age=max(age,elapsed-max(f.since,f.last_dispatch))
            eligible.append((age,g))
        if eligible:
            permitted={id(g) for _,g in eligible}
            goals=[g for g in goals if id(g) in permitted]
            age,oldest=max(eligible,key=lambda item:item[0])
            if age>=90.:
                oldest['overdue']=True
                goals=[oldest]+[g for g in goals if g is not oldest]
        return goals

    @staticmethod
    def utility(goal,walk):
        return goal['reward']/(1.+.24*walk+goal['other_cost'])

    def dispatched(self,goal,elapsed):
        # Keep inherited instrumentation and weak-cue bookkeeping.
        self.active_face=goal.get('object_face')
        if 'family' in goal:
            self.phase='wall' if goal['family']=='wall' else 'interior'
            self.reason='global_nbv_'+goal['family']
            self.last_service[goal['family']]=elapsed
            self.history_goals.append((goal['coverage_odom'],elapsed))
        if self.active_face:
            f=self.tasks.by_key[self.active_face]
            f.attempts+=1;f.last_dispatch=elapsed
            if self.batch_id==goal['box_id']:self.batch_count+=1
            else:self.batch_id,self.batch_since,self.batch_count=goal['box_id'],elapsed,1
        else:self.batch_id,self.batch_count=None,0
        super().dispatched(goal,elapsed)
        self.regions.can_interrupt=False

    def outcome(self,status,reason,elapsed):
        if status not in (4,5) or (status==5 and reason in ('navigation_stalled','action_timeout','bump_contact')):
            cause=reason or f'nav2_status_{status}'
            self.nav_failures[cause]+=1
            if self.active_face:self.tasks.by_key[self.active_face].fail(cause,elapsed)
            self.active_face=None
        elif status==4 and self.active_face:
            # Wait for the real post-arrival sweep and next camera evaluation.
            self.tasks.by_key[self.active_face].fail('arrived_without_sufficient_camera_evidence',elapsed)
            self.active_face=None

    def report(self):
        return dict(surface_cells=self.surfaces,frontier_candidates=self.frontier_count,
                    reachable_goals=len(self.ranked),schedule_reason=self.reason,
                    selection_errors=self.selection_errors,nav_failures=dict(self.nav_failures),termination=self.termination,
                    path_requests=self.paths.queries if self.paths else 0,
                    planner_failures=dict(self.paths.failures) if self.paths else {},
                    objects={str(b.ident):[f.public() for f in b.faces] for b in self.tasks.boxes})
