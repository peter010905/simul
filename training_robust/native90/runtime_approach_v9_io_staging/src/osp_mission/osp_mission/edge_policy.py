"""Inspect explicit occupied/free boundaries, preserving both sides of thin walls."""

import math

import numpy as np
from scipy.ndimage import distance_transform_edt, label, uniform_filter
from scipy.sparse.csgraph import dijkstra

from osp_mission.directional_policy import normals
from osp_mission.policy import graph, wrap
from osp_mission.surface_endpoint_policy import DEFAULT, choose
from osp_mission.coverage_resize import remap
from osp_mission.frontier import reachable_gain
from osp_mission.surface_hits import match


DIRECTIONS=np.array([[-1,0],[0,1],[1,0],[0,-1]])


def boundary_faces(data,resolution,normal_vectors=None):
    """Each face points from an occupied cell into a known free neighbour."""
    occupied=data>50;free=data==0
    neighbours=np.zeros((4,)+data.shape,dtype=bool)
    neighbours[0,1:]=free[:-1]
    neighbours[1,:,:-1]=free[:,1:]
    neighbours[2,:-1]=free[1:]
    neighbours[3,:,1:]=free[:,:-1]
    side,y,x=np.where(neighbours&occupied[None])
    ny,nx=normals(data,resolution) if normal_vectors is None else normal_vectors
    normal_y,normal_x=ny[y,x],nx[y,x]
    dy,dx=DIRECTIONS[side].T
    alignment=normal_y*dy+normal_x*dx
    sign=np.where(alignment<0,-1.,1.)
    normal_y=normal_y*sign;normal_x=normal_x*sign
    ambiguous=np.abs(alignment)<.25
    normal_y[ambiguous]=dy[ambiguous];normal_x[ambiguous]=dx[ambiguous]
    return dict(side=side,y=y,x=x,py=y+.5+.5*dy,px=x+.5+.5*dx,ny=normal_y,nx=normal_x)


def visible(data,from_y,from_x,to_y,to_x):
    """Observed free-space LOS ending just before the boundary, in grid units."""
    if not len(to_y):return np.zeros(0,dtype=bool)
    lengths=np.hypot(to_y-from_y,to_x-from_x)
    stop=np.maximum(0.,1.-.10/np.maximum(lengths,1e-9))
    fractions=stop[:,None]*np.linspace(0,1,max(4,int(lengths.max()*3)+1))
    ys=np.floor(from_y+(to_y[:,None]-from_y)*fractions).astype(int)
    xs=np.floor(from_x+(to_x[:,None]-from_x)*fractions).astype(int)
    inside=(ys>=0)&(ys<data.shape[0])&(xs>=0)&(xs<data.shape[1])
    return np.all(inside&(data[ys.clip(0,data.shape[0]-1),xs.clip(0,data.shape[1]-1)]==0),axis=1)


class EdgeCoverage:
    """Camera evidence is attached to a physical side, not an arbitrary view bin."""

    def __init__(self):
        from osp_mission.normal_cache import AxisNormalCache
        self.geometry=None;self.viewed=None;self.counts=None
        self.last_stamp=-float('inf');self.data=None;self.faces=None
        self.normal_cache=AxisNormalCache()

    def grid(self,data,resolution,origin):
        geometry=(data.shape,resolution,tuple(origin))
        if self.geometry!=geometry:
            if self.geometry is None:
                self.viewed=np.zeros((4,)+data.shape,dtype=bool)
                self.counts=np.zeros_like(self.viewed,dtype=np.uint8)
            else:
                self.viewed=remap(self.viewed,self.data,self.geometry,data,resolution,origin)
                self.counts=remap(self.counts,self.data,self.geometry,data,resolution,origin)
            self.geometry=geometry;self.data=None
        if self.data is None or not np.array_equal(data,self.data):
            faces=boundary_faces(data,resolution,self.normal_cache.update(data,resolution))
            valid=np.zeros_like(self.viewed)
            valid[faces['side'],faces['y'],faces['x']]=True
            self.viewed &= valid;self.counts[~valid]=0
            self.data=data.copy();self.faces=faces
        return self.viewed

    def observe(self,data,resolution,origin,camera,endpoints,stamp,focal,horizontal_fov):
        self.grid(data,resolution,origin)
        if not math.isfinite(stamp) or stamp-self.last_stamp<.15-1e-9:return
        if not math.isfinite(focal) or focal<=0 or not 0<horizontal_fov<math.pi:return
        self.last_stamp=stamp
        points,ys,xs=match(data,resolution,origin,endpoints)
        if not len(points):return
        hit=np.zeros(data.shape,dtype=bool);hit[ys,xs]=True
        f=self.faces
        camera_y=(camera[1]-origin[1])/resolution;camera_x=(camera[0]-origin[0])/resolution
        dy=camera_y-f['py'];dx=camera_x-f['px'];lengths=np.hypot(dy,dx)
        incidence=(dy*f['ny']+dx*f['nx'])/np.maximum(lengths,1e-9)
        relative=(np.arctan2(-dy,-dx)-camera[2]+math.pi)%(2*math.pi)-math.pi
        reliable=hit[f['y'],f['x']]&(lengths*resolution>.25)&(incidence>.6)
        reliable &= np.abs(relative)<horizontal_fov/2
        reliable &= lengths*resolution<.85*focal*.12*incidence/18
        reliable &= lengths*resolution*np.cos(relative)<8.
        indices=np.flatnonzero(reliable)
        clear=visible(data,camera_y,camera_x,f['py'][indices],f['px'][indices])
        indices=indices[clear];side,y,x=f['side'][indices],f['y'][indices],f['x'][indices]
        self.counts[side,y,x]=np.minimum(self.counts[side,y,x]+1,3)
        self.viewed[side,y,x]=self.counts[side,y,x]>=3


def options(data,resolution,origin,robot,heading=0.,visits=(),viewed=None,return_paths=False,point_sampler=None,collect_faces=False,full_scans=False):
    free=data==0;occupied=data>50;unknown=data<0
    clearance=distance_transform_edt(~occupied)*resolution
    safe=free&(clearance>=.32)
    ry=int((robot[1]-origin[1])/resolution);rx=int((robot[0]-origin[0])/resolution)
    if not (0<=ry<data.shape[0] and 0<=rx<data.shape[1]) or not free[ry,rx]:return [],None
    safe[ry,rx]=True;groups,_=label(safe);safe &= groups==groups[ry,rx]
    if safe.sum()<4:return [],None
    distance,previous=dijkstra(graph(safe,resolution),directed=False,indices=ry*data.shape[1]+rx,return_predecessors=True)
    distance=distance.reshape(data.shape)
    f=boundary_faces(data,resolution)
    inspected=np.zeros((4,)+data.shape,dtype=bool) if viewed is None else viewed
    if inspected.shape!=(4,)+data.shape:raise ValueError('Four boundary-face states are required')
    all_faces=f
    unseen=~inspected[f['side'],f['y'],f['x']]
    f={name:values[unseen] for name,values in f.items()}
    gain=reachable_gain(data,resolution,safe)
    valid=safe&np.isfinite(distance)&(distance>.45)&((clearance<3.2)|(gain>.015))
    valid &= uniform_filter(free.astype(float),size=3)>.65
    if point_sampler is None:
        ys,xs=np.where(valid);spacing=max(1,int(1/resolution));buckets={}
        for y,x in zip(ys,xs):
            key=(y//spacing,x//spacing);score=abs(clearance[y,x]-2.)
            if key not in buckets or score<buckets[key][0]:buckets[key]=(score,y,x)
        points=[(y,x) for _,y,x in buckets.values()]
    else:
        points=point_sampler(data,resolution,origin,robot,safe,distance,all_faces,inspected)
    result=[]
    for y,x in points:
        px=origin[0]+(x+.5)*resolution;py=origin[1]+(y+.5)*resolution
        revisit=sum(math.exp(-((px-a)**2+(py-b)**2)/.5) for a,b in visits[-100:])
        if revisit>1.5:continue
        dy=y+.5-f['py'];dx=x+.5-f['px'];lengths=np.hypot(dy,dx)
        incidence=(dy*f['ny']+dx*f['nx'])/np.maximum(lengths,1e-9)
        indices=np.flatnonzero((incidence>.6)&(lengths*resolution<3.7*incidence))
        clear=visible(data,y+.5,x+.5,f['py'][indices],f['px'][indices])
        indices=indices[clear]
        counted=indices
        yaw=math.atan2(py-robot[1],px-robot[0]);angle=2*math.pi if full_scans else 2*math.pi/3;count=0.
        if len(indices) and full_scans:
            count=len(indices)*resolution
        elif len(indices):
            angles=np.arctan2(-dy[indices],-dx[indices]);sectors=np.arange(-math.pi,math.pi,math.pi/8)
            counts=np.sum(np.abs((angles[:,None]-sectors+math.pi)%(2*math.pi)-math.pi)<math.radians(55),axis=0)
            best=int(np.argmax(counts));count=float(counts[best])*resolution;yaw=float(sectors[best])-math.pi/3
            counted=indices[np.abs((angles-sectors[best]+math.pi)%(2*math.pi)-math.pi)<math.radians(55)]
            if len(indices)>counts[best]*1.5:
                count=len(indices)*resolution;angle=2*math.pi;yaw=math.atan2(py-robot[1],px-robot[0])
                counted=indices
        if count==0 and gain[y,x]<.015:continue
        result.append(dict(x=px,y=py,yaw=wrap(yaw),scan_angle=angle,cell=(int(y),int(x)),distance=float(distance[y,x]),
            features=np.array([count,gain[y,x]*5,distance[y,x],abs(wrap(yaw-heading)),revisit])))
        if collect_faces:
            result[-1]['visible_faces']=np.column_stack((f['side'][counted],f['y'][counted],f['x'][counted]))
    return result,previous if return_paths else None


def load_policy(filename):
    if not filename:return DEFAULT.copy()
    import json
    from pathlib import Path
    payload=json.loads(Path(filename).expanduser().read_text())
    weights=np.asarray(payload['parameters'],dtype=float)
    if payload.get('planner')!='edge' or weights.shape!=(5,) or not np.isfinite(weights).all() or np.any(weights<=0):
        raise ValueError('Invalid boundary-face policy')
    return weights
