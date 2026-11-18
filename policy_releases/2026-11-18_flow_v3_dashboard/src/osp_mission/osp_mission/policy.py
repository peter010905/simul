"""Observable-map-only hierarchical search policy shared by training and ROS."""
import json
import math
from pathlib import Path
import numpy as np
from scipy.ndimage import distance_transform_edt, label, uniform_filter, binary_dilation, gaussian_filter
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

FEATURES=['unknown_gain','near_surfaces','travel_cost','turn_cost','revisits','clearance','unobserved_surfaces','frontier']
BASELINE=np.array([1.,.8,-1.2,-.2,-2.,.2,1.5,.4,3.])


def wrap(a): return (a+math.pi)%(2*math.pi)-math.pi


def load_policy(filename):
    if not filename: return BASELINE.copy()
    cfg=json.loads(Path(filename).expanduser().read_text())
    if cfg.get('features')!=FEATURES or len(cfg['parameters'])!=len(BASELINE): raise ValueError('Incompatible policy schema')
    weights=np.array(cfg['parameters'],dtype=float)
    if not np.isfinite(weights).all(): raise ValueError('Non-finite policy parameters')
    return weights


def scan_angle(weights):
    # Learn observation time, while retaining at least a 120 degree inspection.
    return float(2*math.pi/3+(4*math.pi/3)/(1+math.exp(-float(np.clip(weights[-1],-8,8)))))


def graph(safe,resolution):
    rows,cols=safe.shape; ids=np.arange(rows*cols).reshape(rows,cols)
    ii=[]; jj=[]; values=[]
    for dy,dx in [(0,1),(1,0),(1,1),(1,-1)]:
        y0=max(0,-dy); y1=rows-max(0,dy); x0=max(0,-dx); x1=cols-max(0,dx)
        a=safe[y0:y1,x0:x1]; b=safe[y0+dy:y1+dy,x0+dx:x1+dx]; valid=a&b
        if dy and dx:
            valid &= safe[y0+dy:y1+dy,x0:x1] & safe[y0:y1,x0+dx:x1+dx]
        src=ids[y0:y1,x0:x1][valid]; dst=ids[y0+dy:y1+dy,x0+dx:x1+dx][valid]
        ii.extend([src,dst]); jj.extend([dst,src]); values.extend([np.full(len(src),resolution*math.hypot(dx,dy))]*2)
    return csr_matrix((np.concatenate(values),(np.concatenate(ii),np.concatenate(jj))),shape=(rows*cols,rows*cols))


def options(data,resolution,origin,robot,heading=0.,visits=(),viewed=None,return_paths=False):
    free=data==0; occupied=data>50; unknown=data<0
    clearance=distance_transform_edt(~occupied)*resolution
    safe=free&(clearance>=.29)
    ry=int((robot[1]-origin[1])/resolution); rx=int((robot[0]-origin[0])/resolution)
    if not (0<=ry<data.shape[0] and 0<=rx<data.shape[1]): return [],None
    # Permit the current robot cell and its immediate connected exit to handle coarse grids.
    if not free[ry,rx]: return [],None
    safe[ry,rx]=True
    groups,_=label(safe); safe &= groups==groups[ry,rx]
    if safe.sum()<4: return [],None
    network=graph(safe,resolution)
    distances,predecessors=dijkstra(network,directed=False,indices=ry*data.shape[1]+rx,return_predecessors=True)
    distances=distances.reshape(data.shape)
    surface_distance,nearest=distance_transform_edt(~occupied,return_indices=True)
    # An occupied cell seen by LiDAR is not necessarily inspected by the camera.
    inspected=np.zeros_like(free) if viewed is None else viewed
    surface=occupied & binary_dilation(free)
    inspected=binary_dilation(inspected,iterations=max(1,int(.18/resolution)))
    unseen_surfaces=surface & ~inspected
    radius=max(3,int(4/resolution)|1)
    gain=uniform_filter(unknown.astype(float),size=radius,mode='constant')
    surfaces=uniform_filter(surface.astype(float),size=radius,mode='constant')
    unseen=uniform_filter(unseen_surfaces.astype(float),size=max(3,int(2.6/resolution)|1),mode='constant')
    spacing=max(1,int(.85/resolution))
    destination_known=uniform_filter(free.astype(float),size=max(3,int(.5/resolution)|1))>.70
    ys,xs=np.where(safe & destination_known & np.isfinite(distances) & (distances>.4))
    buckets={}
    for y,x in zip(ys,xs):
        key=(int(y)//spacing,int(x)//spacing)
        desirability=clearance[y,x]
        if key not in buckets or desirability>buckets[key][0]: buckets[key]=(desirability,int(y),int(x))
    result=[]
    for _,y,x in buckets.values():
        px=origin[0]+(x+.5)*resolution; py=origin[1]+(y+.5)*resolution
        revisit=sum(math.exp(-((px-a)**2+(py-b)**2)/1.5) for a,b in visits[-80:])
        if revisit>2.5: continue
        oy,ox=nearest[:,y,x]
        yaw=math.atan2(oy-y,ox-x) if surface_distance[y,x]*resolution<3.5 else math.atan2(py-robot[1],px-robot[0])
        turn=abs(wrap(math.atan2(py-robot[1],px-robot[0])-heading))/math.pi
        feature=np.array([gain[y,x]*4,min(surfaces[y,x]*10,1),distances[y,x]/10,turn,
                          min(revisit,3),min(clearance[y,x],1.5)/1.5,min(unseen[y,x]*10,1),float(gain[y,x]>.02)])
        result.append(dict(x=px,y=py,yaw=yaw,features=feature,distance=float(distances[y,x]),cell=(y,x)))
    return result,predecessors if return_paths else None


def choose(candidates,weights):
    if not candidates: return None
    scores=[float(c['features']@weights[:len(FEATURES)]) for c in candidates]
    return candidates[int(np.argmax(scores))]
