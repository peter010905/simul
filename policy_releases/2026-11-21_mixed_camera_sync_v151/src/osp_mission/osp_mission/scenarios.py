"""Seeded obstacle curricula and shared collision geometry (metres)."""
import argparse
import math
from pathlib import Path
import numpy as np
from scipy.ndimage import distance_transform_edt, label
import yaml

WIDTH, HEIGHT = 20.15, 11.35


def fixtures():
    return ([dict(type='box',x=x,y=11.15,sx=.5,sy=.35,yaw=0.,height=1.4)
             for x in [.5,4.,8.,12.,16.,19.3]] +
            [dict(type='box',x=x,y=.2,sx=.5,sy=.4,yaw=0.,height=1.4)
             for x in [.5,6.4,15.9,17.6]])


def rasterize(cfg, resolution=.15, include_supports=True):
    rows=int(math.ceil(HEIGHT/resolution)); cols=int(math.ceil(WIDTH/resolution))
    yy,xx=np.indices((rows,cols)); x=(xx+.5)*resolution; y=(yy+.5)*resolution
    # Include cells intersecting a wall, not only cells whose centre lies in it.
    # At 0.20 m resolution the old centre test omitted the west/south 0.12 m walls.
    margin=.06+resolution/2
    occ=(x<margin)|(x>WIDTH-margin)|(y<margin)|(y>HEIGHT-margin)
    objects=fixtures()+list(cfg.get('obstacles',[]))
    if include_supports:
        for t in cfg.get('targets',[]):
            a=t['yaw']; objects.append(dict(type='box',x=t['x']-.05*math.cos(a),
                y=t['y']-.05*math.sin(a),sx=.08,sy=t['size']*1.25,yaw=a,height=.9))
    for o in objects:
        dx=x-o['x']; dy=y-o['y']
        if o['type']=='cylinder': inside=dx*dx+dy*dy<=(o['radius']+resolution*.4)**2
        else:
            a=o.get('yaw',0.); u=dx*math.cos(a)+dy*math.sin(a); v=-dx*math.sin(a)+dy*math.cos(a)
            inside=(np.abs(u)<=o['sx']/2+resolution*.4)&(np.abs(v)<=o['sy']/2+resolution*.4)
        occ|=inside
    return occ


def reachable(cfg,resolution=.15):
    occ=rasterize(cfg,resolution)
    safe=distance_transform_edt(~occ)*resolution>=.30
    regions,_=label(safe)
    x,y,_=cfg['start']; row,col=int(y/resolution),int(x/resolution)
    component=regions[row,col]
    return (regions==component)&safe if component else np.zeros_like(safe)


def valid(cfg,resolution=.15):
    component=reachable(cfg,resolution)
    if component.sum()*resolution**2<100: return False
    for t in cfg['targets']:
        # Must have a clear, reachable front-facing approach and line of sight.
        a=t['yaw']; ok=False
        for d in [.7,1.,1.4]:
            x=t['x']+d*math.cos(a); y=t['y']+d*math.sin(a)
            row,col=int(y/resolution),int(x/resolution)
            if not (0<=row<component.shape[0] and 0<=col<component.shape[1] and component[row,col]): continue
            occ=rasterize(cfg,resolution)
            ds=np.linspace(.20,d,15)
            xs=((t['x']+ds*math.cos(a))/resolution).astype(int)
            ys=((t['y']+ds*math.sin(a))/resolution).astype(int)
            if not occ[ys,xs].any(): ok=True; break
        if not ok: return False
    return True


def make_scenario(seed, count=None):
    rng=np.random.default_rng(seed)
    count=int(count if count is not None else [8,16,24,32][seed%4])
    for attempt in range(60):
        cfg={'seed':int(seed),'randomize':False,'difficulty':count,'start':[1.2,1.3,0.], 'targets':[], 'obstacles':[]}
        for j in range(count):
            kind=['box','cylinder','rack','partition'][j%4]
            for _ in range(100):
                x=float(rng.uniform(2.3,18.5)); y=float(rng.uniform(1.4,9.8))
                sx,sy=(float(rng.uniform(.4,1.0)),float(rng.uniform(.4,1.0))) if kind=='box' else (
                    (float(rng.uniform(1.,2.4)),.55) if kind=='rack' else (float(rng.uniform(1.4,3.0)),.16))
                radius=float(rng.uniform(.18,.40)); extent=radius if kind=='cylinder' else math.hypot(sx,sy)/2
                if any(math.hypot(x-o['x'],y-o['y']) < extent+o['extent']+.30 for o in cfg['obstacles']): continue
                o=dict(type='cylinder' if kind=='cylinder' else 'box',kind=kind,x=x,y=y,
                       sx=sx,sy=sy,radius=radius,yaw=float(rng.uniform(-math.pi,math.pi)),height=float(rng.uniform(.65,1.3)),extent=extent)
                cfg['obstacles'].append(o); break
        ids=rng.choice(1024,3,replace=False)
        carriers=rng.choice(len(cfg['obstacles']),3,replace=False)
        cfg['task']='hidden_obstacle_aruco_v2'
        for j,index in enumerate(carriers):
            o=cfg['obstacles'][int(index)]
            away=math.atan2(o['y']-cfg['start'][1],o['x']-cfg['start'][0])
            if o['type']=='cylinder':
                a=away; extent=o['radius']; face_width=2*o['radius']
            else:
                faces=[(o['yaw'],o['sx']/2,o['sy']), (o['yaw']+math.pi,o['sx']/2,o['sy']),
                       (o['yaw']+math.pi/2,o['sy']/2,o['sx']), (o['yaw']-math.pi/2,o['sy']/2,o['sx'])]
                faces=[v for v in faces if v[2]>=.30]
                a,extent,face_width=max(faces,key=lambda v:math.cos(v[0]-away))
            offset=extent+.055
            x=o['x']+offset*math.cos(a); y=o['y']+offset*math.sin(a)
            cfg['targets'].append(dict(id=int(ids[j]),x=float(x),y=float(y),yaw=float(a),
                size=float(rng.uniform(.16,min(.32,face_width*.70))),carrier_obstacle=int(index),hidden_from_start=True))
        if valid(cfg):
            for o in cfg['obstacles']: o.pop('extent')
            return cfg
    raise RuntimeError(f'Cannot generate accessible seed {seed}, count={count}')


def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',default='scenarios_hidden'); p.add_argument('--train',type=int,default=120)
    p.add_argument('--validation',type=int,default=32); p.add_argument('--test',type=int,default=48)
    a=p.parse_args(); manifest={}
    for split,start,count in [('train',0,a.train),('validation',10000,a.validation),('test',20000,a.test)]:
        dest=Path(a.output)/split; dest.mkdir(parents=True,exist_ok=True)
        manifest[split]=list(range(start,start+count))
        for seed in manifest[split]:
            (dest/f'{seed:05d}.yaml').write_text(yaml.safe_dump(make_scenario(seed),sort_keys=False))
        print(f'{split}: {count} accessible layouts',flush=True)
    (Path(a.output)/'manifest.yaml').write_text(yaml.safe_dump(manifest))

if __name__=='__main__': main()
