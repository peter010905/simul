"""Evaluator-only concave arenas: C/ㄷ, U, and three-pass zigzag corridors.

Never imported by the policy. Existing frozen space generators are unchanged.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.sparse.csgraph import dijkstra
import yaml

from osp_mission.policy import graph
from osp_mission.space_geometry import wall, polygon_area, rasterize_space, valid_space, triangulate

SHAPES=('c_shape','u_shape','zigzag')


def rectangle_union(rectangles):
    """Trace one hole-free orthogonal polygon from a connected rectangle union."""
    xs=sorted({r[i] for r in rectangles for i in (0,2)})
    ys=sorted({r[i] for r in rectangles for i in (1,3)})
    edges=set()
    for x0,x1 in zip(xs,xs[1:]):
        for y0,y1 in zip(ys,ys[1:]):
            if not any(a<=(x0+x1)/2<=c and b<=(y0+y1)/2<=d for a,b,c,d in rectangles):continue
            points=[(x0,y0),(x1,y0),(x1,y1),(x0,y1)]
            for a,b in zip(points,points[1:]+points[:1]):
                if (b,a) in edges:edges.remove((b,a))
                else:edges.add((a,b))
    next_point=dict(edges)
    if len(next_point)!=len(edges):raise ValueError('Ambiguous boundary')
    start=min(next_point);p=start;outline=[]
    for _ in range(len(edges)):
        outline.append(p);p=next_point[p]
        if p==start:break
    if p!=start or len(outline)!=len(edges):raise ValueError('Disconnected union or hole')
    simplified=[]
    for i,p in enumerate(outline):
        a=outline[i-1];b=outline[(i+1)%len(outline)]
        if (p[0]-a[0])*(b[1]-p[1])-(p[1]-a[1])*(b[0]-p[0])!=0:simplified.append(list(p))
    return simplified


def make_challenge_arena(rng,shape):
    if shape not in SHAPES:raise ValueError(shape)
    width=float(rng.uniform(17.,22.));band=float(rng.uniform(2.8,3.8))
    if shape=='zigzag':
        height=5*band
        rectangles=[(0,0,width,band),(width-band,band,width,2*band),
            (0,2*band,width,3*band),(0,3*band,band,4*band),(0,4*band,width,5*band)]
        polygon=rectangle_union(rectangles)
        spawn=[band/2,band/2,0.];far=[width-band/2,height-band/2]
    else:
        height=float(rng.uniform(13.,17.))
        if shape=='c_shape':
            polygon=[[0,0],[width,0],[width,band],[band,band],
                     [band,height-band],[width,height-band],[width,height],[0,height]]
            spawn=[width-band/2,band/2,math.pi];far=[width-band/2,height-band/2]
        else:
            polygon=[[0,0],[width,0],[width,height],[width-band,height],
                     [width-band,band],[band,band],[band,height],[0,height]]
            spawn=[band/2,height-band/2,-math.pi/2];far=[width-band/2,height-band/2]
    triangulate(polygon)  # Reject self-intersections and invalid winding.
    arena=dict(schema_version=1,shape=shape,width_m=width,height_m=height,
        polygon=polygon,walls=[wall(a,b,boundary=True) for a,b in zip(polygon,polygon[1:]+polygon[:1])],
        doorways=[],area_m2=polygon_area(polygon),pdf_fixtures=False,
        nominal_corridor_width_m=band,challenge_version=1)
    return arena,spawn,far


def make_challenge_scene(seed,count=12,profile='hidden_obstacles',shape='zigzag'):
    if profile!='hidden_obstacles':raise ValueError('Challenge targets must be hidden on obstacles')
    if not 6<=count<=32:raise ValueError('Use 6..32 obstacles')
    rng=np.random.default_rng(seed)
    for attempt in range(120):
        arena,start,far=make_challenge_arena(rng,shape)
        cfg=dict(seed=int(seed),randomize=False,task='concave_hidden_search_v1',arena=arena,
            start=start,difficulty=int(count),actual_obstacles=0,mount_profile=profile,obstacles=[],targets=[])
        base=rasterize_space(cfg,.10);clear=distance_transform_edt(~base)*.10
        ys,xs=np.where(clear>.65)
        for index in range(count):
            kind=('box','cylinder','rack','partition')[index%4]
            for _ in range(200):
                pick=int(rng.integers(len(xs)));x=float((xs[pick]+.5)*.10);y=float((ys[pick]+.5)*.10)
                radius=float(rng.uniform(.18,.32))
                sx,sy=(float(rng.uniform(.4,.8)),float(rng.uniform(.4,.8))) if kind=='box' else (float(rng.uniform(.85,1.5)),.35 if kind=='rack' else .16)
                extent=radius if kind=='cylinder' else math.hypot(sx,sy)/2
                if clear[ys[pick],xs[pick]]<extent+.42:continue
                if math.hypot(x-start[0],y-start[1])<extent+1.:continue
                if any(math.hypot(x-o['x'],y-o['y'])<extent+o['extent']+.55 for o in cfg['obstacles']):continue
                cfg['obstacles'].append(dict(type='cylinder' if kind=='cylinder' else 'box',kind=kind,
                    x=x,y=y,radius=radius,sx=sx,sy=sy,yaw=float(rng.uniform(-math.pi,math.pi)),
                    height=float(rng.uniform(.75,1.3)),extent=extent));break
        if len(cfg['obstacles'])!=count:continue  # Do not silently lower the requested challenge.
        distances=dijkstra(graph(clear>=.34,.10),directed=False,
                           indices=int(start[1]/.10)*base.shape[1]+int(start[0]/.10)).reshape(base.shape)
        ranked=sorted(range(count),key=lambda i:distances[int(cfg['obstacles'][i]['y']/.10),int(cfg['obstacles'][i]['x']/.10)])
        # Carriers are spread through three different sections of the corridor.
        carriers=[ranked[min(count-1,int(count*f))] for f in (.2,.5,.85)]
        for ident,index in zip(rng.choice(1024,3,replace=False),carriers):
            o=cfg['obstacles'][index];away=math.atan2(o['y']-start[1],o['x']-start[0])
            if o['type']=='cylinder':a,extent,width=away,o['radius'],2*o['radius']
            else:
                faces=[(o['yaw'],o['sx']/2,o['sy']),(o['yaw']+math.pi,o['sx']/2,o['sy']),
                    (o['yaw']+math.pi/2,o['sy']/2,o['sx']),(o['yaw']-math.pi/2,o['sy']/2,o['sx'])]
                a,extent,width=max((f for f in faces if f[2]>=.25),key=lambda f:math.cos(f[0]-away))
            cfg['targets'].append(dict(id=int(ident),x=o['x']+(extent+.055)*math.cos(a),
                y=o['y']+(extent+.055)*math.sin(a),yaw=a,size=float(rng.uniform(.10,min(.26,width*.65))),
                mount='obstacle',carrier_obstacle=index,hidden_from_start=True))
        if any((start[0]-t['x'])*math.cos(t['yaw'])+(start[1]-t['y'])*math.sin(t['yaw'])>=-.2 for t in cfg['targets']):continue
        if not (valid_space(cfg,.10) and valid_space(cfg,.15)):continue
        for o in cfg['obstacles']:o.pop('extent')
        cfg['actual_obstacles']=count
        cfg['challenge']=dict(version=1,far_endpoint=far,nominal_corridor_width_m=arena['nominal_corridor_width_m'],
            required_turns=4 if shape=='zigzag' else 2,policy_receives_ground_truth=False,generation_attempt=attempt)
        return cfg
    raise RuntimeError(f'No accessible challenge for seed={seed}, count={count}, shape={shape}')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--per-shape',type=int,default=4);p.add_argument('--seed',type=int,default=1192300000)
    p.add_argument('--obstacles',type=int,default=12);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False);rows=[]
    for k,shape in enumerate(SHAPES):
        for i in range(a.per_shape):
            cfg=make_challenge_scene(a.seed+k*10000+i*101,a.obstacles,shape=shape)
            path=a.output/f'{shape}_{cfg["seed"]}.yaml';path.write_text(yaml.safe_dump(cfg,sort_keys=False))
            rows.append(dict(file=str(path.resolve()),seed=cfg['seed'],shape=shape,obstacles=len(cfg['obstacles']),
                             sha256=hashlib.sha256(path.read_bytes()).hexdigest(),reachable=valid_space(cfg,.10)))
            print(f'{shape}: seed {cfg["seed"]}, {len(cfg["obstacles"])} obstacles',flush=True)
    (a.output/'manifest.json').write_text(json.dumps(dict(evidence_type='offline_geometry_only',native_success=False,episodes=rows),indent=2)+'\n')

if __name__=='__main__':main()
