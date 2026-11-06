"""Incidence-aware surface coverage; unknown targets and obstacle geometry stay private."""
import math
import numpy as np
from scipy.ndimage import distance_transform_edt, binary_dilation, uniform_filter, label, gaussian_filter
from scipy.sparse.csgraph import dijkstra
from osp_mission.policy import graph,wrap
from osp_mission.frontier import reachable_gain

# gain / travel weighting is tuned independently of hidden target identities.
DEFAULT=np.array([1.,2.,1.,.3,1.])


def normals(data,resolution):
    field=gaussian_filter((data>50).astype(float),sigma=max(.7,.20/resolution))
    gy,gx=np.gradient(field)
    scale=np.hypot(gx,gy)
    return -gy/np.maximum(scale,1e-9),-gx/np.maximum(scale,1e-9)


def options(data,resolution,origin,robot,heading=0.,visits=(),viewed=None,return_paths=False,preferred_clearance=2.):
    free=data==0;occupied=data>50;unknown=data<0
    clearance=distance_transform_edt(~occupied)*resolution
    safe=free&(clearance>=.32)
    ry=int((robot[1]-origin[1])/resolution);rx=int((robot[0]-origin[0])/resolution)
    if not(0<=ry<data.shape[0] and 0<=rx<data.shape[1]) or not free[ry,rx]:return [],None
    safe[ry,rx]=True;groups,_=label(safe);safe &= groups==groups[ry,rx]
    if safe.sum()<4:return [],None
    distance,previous=dijkstra(graph(safe,resolution),directed=False,indices=ry*data.shape[1]+rx,return_predecessors=True)
    distance=distance.reshape(data.shape)
    surface=occupied&binary_dilation(free)
    inspected=np.zeros_like(free) if viewed is None else viewed
    unseen=surface&~inspected
    sy,sx=np.where(unseen)
    normal_y,normal_x=normals(data,resolution)
    gain=reachable_gain(data,resolution,safe)
    # Keep near-surface viewpoints rather than the highest-clearance point per tile.
    valid=safe&np.isfinite(distance)&(distance>.45)&((clearance<3.2)|(gain>.015))
    valid &= uniform_filter(free.astype(float),size=3)>.65
    ys,xs=np.where(valid);spacing=max(1,int(1.0/resolution));buckets={}
    for y,x in zip(ys,xs):
        k=(y//spacing,x//spacing);score=abs(clearance[y,x]-preferred_clearance)
        if k not in buckets or score<buckets[k][0]:buckets[k]=(score,y,x)
    result=[]
    for _,y,x in buckets.values():
        px=origin[0]+(x+.5)*resolution;py=origin[1]+(y+.5)*resolution
        revisit=sum(math.exp(-((px-a)**2+(py-b)**2)/.5) for a,b in visits[-100:])
        if revisit>1.5:continue
        distances=np.hypot(sy-y,sx-x)
        incidence=((y-sy)*normal_y[sy,sx]+(x-sx)*normal_x[sy,sx])/np.maximum(distances,1e-9)
        nearby=(incidence>.6)&(distances*resolution<3.7*incidence)
        ty,tx=sy[nearby],sx[nearby];angle=2*math.pi/3;count=0;yaw=math.atan2(py-robot[1],px-robot[0])
        if len(ty):
            # Exclude one endpoint cell, not a fixed percentage of the ray.
            # A percentage rejects close views when the last sample rounds into
            # the target surface's occupied cell.
            lengths=np.hypot(ty-y,tx-x)
            stop=np.maximum(0.,1.-1.01/np.maximum(lengths,1e-9))
            fractions=stop[:,None]*np.linspace(0,1,max(4,int(3.7/resolution)*2))
            ly=np.rint(y+(ty[:,None]-y)*fractions).astype(int)
            lx=np.rint(x+(tx[:,None]-x)*fractions).astype(int)
            visible=np.all(data[ly,lx]==0,axis=1);ty,tx=ty[visible],tx[visible]
            if len(ty):
                angles=np.arctan2(ty-y,tx-x)
                sectors=np.arange(-math.pi,math.pi,math.pi/8)
                counts=np.sum(np.abs((angles[:,None]-sectors+math.pi)%(2*math.pi)-math.pi)<math.radians(55),axis=0)
                best=int(np.argmax(counts));count=float(counts[best])*resolution;yaw=float(sectors[best])-math.pi/3
                if len(ty)>counts[best]*1.5:
                    count=float(len(ty))*resolution;angle=2*math.pi
                    yaw=math.atan2(py-robot[1],px-robot[0])
        if count==0 and gain[y,x]<.015:continue
        result.append(dict(x=px,y=py,yaw=wrap(yaw),scan_angle=angle,cell=(int(y),int(x)),distance=float(distance[y,x]),
            features=np.array([count,gain[y,x]*5,distance[y,x],abs(wrap(yaw-heading)),revisit])))
    return result,previous if return_paths else None


def choose(candidates,weights=DEFAULT):
    if not candidates:return None
    def score(c):
        surface,frontier,distance,turn,revisit=c['features']
        return (weights[0]*surface+weights[1]*frontier)/(weights[2]*distance/.26+weights[3]*turn/.5+5.)-weights[4]*revisit*.1
    return max(candidates,key=score)


def load_policy(filename):
    return load_configuration(filename)[0]


def load_configuration(filename):
    """Validate weights and optional observation/deadline settings together."""
    import json
    from pathlib import Path
    cfg=dict(planner='surface_endpoint',parameters=DEFAULT.tolist()) if not filename else json.loads(Path(filename).expanduser().read_text())
    if not isinstance(cfg,dict):raise ValueError('Policy must be an object')
    weights=np.asarray(cfg['parameters'],dtype=float)
    if cfg.get('planner')!='surface_endpoint' or weights.shape!=(5,) or not np.isfinite(weights).all() or np.any(weights<=0):
        raise ValueError('Invalid inspection policy')
    settings=cfg.get('settings',{})
    if not isinstance(settings,dict):raise ValueError('Policy settings must be an object')
    clearance=settings.get('view_standoff',2.)
    deadline=settings.get('deadline_aware',False)
    if isinstance(clearance,bool) or not isinstance(clearance,(int,float)) or not math.isfinite(clearance) or not .5<=clearance<=3.:
        raise ValueError('view_standoff must be finite and between 0.5 and 3 metres')
    if not isinstance(deadline,bool):raise ValueError('deadline_aware must be a boolean')
    return weights,dict(view_standoff=float(clearance),deadline_aware=deadline)
