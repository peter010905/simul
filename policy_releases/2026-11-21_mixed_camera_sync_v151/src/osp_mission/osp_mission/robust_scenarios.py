"""Dense, mixed-mount robustness scenarios; targets remain environment-private."""
import argparse
import copy
import math
from pathlib import Path
import numpy as np
import yaml
from osp_mission.scenarios import make_scenario, valid

PROFILES=('walls','mixed','obstacles','hidden_obstacles')
COUNTS=(8,12,16,24,32)


def make_robust(seed,count=12,profile='hidden_obstacles'):
    if profile not in PROFILES:raise ValueError(profile)
    rng=np.random.default_rng(seed)
    for attempt in range(80):
        cfg=copy.deepcopy(make_scenario(seed+attempt*10000000,count))
        cfg['seed']=int(seed);cfg['task']='robust_search_v1';cfg['mount_profile']=profile
        cfg['start']=list([(1.2,1.3,0.),(18.8,1.3,math.pi),(1.2,10.,0.),(18.8,10.,math.pi)][int(rng.integers(4))])
        cfg['start'][2]=float(rng.uniform(-math.pi,math.pi))
        wall_count={'walls':3,'mixed':1,'obstacles':0,'hidden_obstacles':0}[profile]
        walls=[(float(rng.uniform(2.,18.)),.13,math.pi/2),(float(rng.uniform(2.,18.)),11.22,-math.pi/2),
               (.13,float(rng.uniform(2.,9.)),0.),(20.02,float(rng.uniform(2.,9.)),math.pi)]
        targets=[]
        for i in rng.choice(4,wall_count,replace=False):
            x,y,a=walls[i];targets.append(dict(x=x,y=y,yaw=a,size=float(rng.uniform(.12,.30)),mount='wall'))
        for i in rng.choice(len(cfg['obstacles']),3-wall_count,replace=False):
            o=cfg['obstacles'][i];away=math.atan2(o['y']-cfg['start'][1],o['x']-cfg['start'][0])
            if o['type']=='cylinder':
                a=away if profile=='hidden_obstacles' else float(rng.uniform(-math.pi,math.pi))
                d=o['radius'];width=2*o['radius']
            else:
                faces=[(o['yaw'],o['sx']/2,o['sy']),(o['yaw']+math.pi,o['sx']/2,o['sy']),
                       (o['yaw']+math.pi/2,o['sy']/2,o['sx']),(o['yaw']-math.pi/2,o['sy']/2,o['sx'])]
                faces=[f for f in faces if f[2]>=.25]
                a,d,width=max(faces,key=lambda f:math.cos(f[0]-away)) if profile=='hidden_obstacles' else faces[int(rng.integers(len(faces)))]
            targets.append(dict(x=o['x']+(d+.055)*math.cos(a),y=o['y']+(d+.055)*math.sin(a),yaw=a,
                size=float(rng.uniform(.12,min(.30,width*.7))),mount='obstacle',carrier_obstacle=int(i)))
        for t,ident in zip(targets,rng.choice(1024,3,replace=False)):t['id']=int(ident)
        cfg['targets']=targets
        if valid(cfg):
            cfg['actual_obstacles']=len(cfg['obstacles'])
            # Only used by evaluation, never passed to the planning function.
            cfg['sensor_stress']=dict(detection_dropout=float(rng.choice([.08,.20])),speed_limit=float(rng.choice([.20,.23,.26])))
            return cfg
    raise RuntimeError(f'No accessible robust layout: {seed}/{count}/{profile}')


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',default='scenarios_robust');a=p.parse_args()
    root=Path(a.output);manifest={}
    for split,start,n in [('train',8100000,100),('validation',8200000,80),('test',8300000,100)]:
        out=root/split;out.mkdir(parents=True,exist_ok=True);manifest[split]=[]
        for i in range(n):
            c=make_robust(start+i,COUNTS[i%5],PROFILES[(i//5)%4])
            (out/f'{start+i}.yaml').write_text(yaml.safe_dump(c,sort_keys=False));manifest[split].append(start+i)
        print(split,n,flush=True)
    (root/'manifest.yaml').write_text(yaml.safe_dump(manifest))

if __name__=='__main__':main()
