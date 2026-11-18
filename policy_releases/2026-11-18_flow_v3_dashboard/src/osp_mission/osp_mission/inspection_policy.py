"""Reachable camera inspection viewpoints from observed occupancy only."""
import math
import numpy as np
from scipy.ndimage import distance_transform_edt, binary_dilation, uniform_filter, label
from scipy.sparse.csgraph import dijkstra
from osp_mission.policy import graph,wrap

# gain / travel weighting is tuned independently of hidden target identities.
DEFAULT=np.array([1.,2.,1.,.3,1.])


def options(data,resolution,origin,robot,heading=0.,visits=(),viewed=None,return_paths=False):
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
    gain=uniform_filter(unknown.astype(float),size=max(3,int(3/resolution)|1))
    # Keep near-surface viewpoints rather than the highest-clearance point per tile.
    valid=safe&np.isfinite(distance)&(distance>.45)&((clearance<2.4)|(gain>.015))
    valid &= uniform_filter(free.astype(float),size=3)>.65
    ys,xs=np.where(valid);spacing=max(1,int(1.0/resolution));buckets={}
    for y,x in zip(ys,xs):
        k=(y//spacing,x//spacing);score=abs(clearance[y,x]-1.7)
        if k not in buckets or score<buckets[k][0]:buckets[k]=(score,y,x)
    result=[]
    for _,y,x in buckets.values():
        px=origin[0]+(x+.5)*resolution;py=origin[1]+(y+.5)*resolution
        revisit=sum(math.exp(-((px-a)**2+(py-b)**2)/.5) for a,b in visits[-100:])
        if revisit>1.5:continue
        nearby=((sy-y)**2+(sx-x)**2)*resolution**2<2.5**2
        ty,tx=sy[nearby],sx[nearby];count=0;yaw=math.atan2(py-robot[1],px-robot[0])
        if len(ty):
            fractions=np.linspace(0,.9,max(4,int(2.5/resolution)*2))
            ly=np.rint(y+(ty[:,None]-y)*fractions).astype(int)
            lx=np.rint(x+(tx[:,None]-x)*fractions).astype(int)
            visible=np.all(data[ly,lx]==0,axis=1);ty,tx=ty[visible],tx[visible]
            if len(ty):
                angles=np.arctan2(ty-y,tx-x)
                sectors=np.arange(-math.pi,math.pi,math.pi/8)
                counts=np.sum(np.abs((angles[:,None]-sectors+math.pi)%(2*math.pi)-math.pi)<math.radians(55),axis=0)
                best=int(np.argmax(counts));count=float(counts[best])*resolution;yaw=float(sectors[best])
        if count==0 and gain[y,x]<.015:continue
        result.append(dict(x=px,y=py,yaw=wrap(yaw-math.pi/3),cell=(int(y),int(x)),distance=float(distance[y,x]),
            features=np.array([count,gain[y,x]*5,distance[y,x],abs(wrap(yaw-heading)),revisit])))
    return result,previous if return_paths else None


def choose(candidates,weights=DEFAULT):
    if not candidates:return None
    def score(c):
        surface,frontier,distance,turn,revisit=c['features']
        return (weights[0]*surface+weights[1]*frontier)/(weights[2]*distance/.26+weights[3]*turn/.5+5.)-weights[4]*revisit*.1
    return max(candidates,key=score)


def load_policy(filename):
    if not filename:return DEFAULT.copy()
    import json
    from pathlib import Path
    cfg=json.loads(Path(filename).expanduser().read_text())
    values=np.asarray(cfg['parameters'],dtype=float)
    if cfg.get('planner')!='inspection' or values.shape!=(5,) or not np.isfinite(values).all() or np.any(values<=0):
        raise ValueError('Invalid inspection policy')
    return values
