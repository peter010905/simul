"""PDF-inspired fixtures, not a claim about the undisclosed demonstration layout."""
import argparse
import math
from pathlib import Path
import numpy as np
import yaml
from osp_mission.scenarios import valid


def make_assignment(seed, count=4, vary_start=False):
    rng=np.random.default_rng(seed)
    centers=[(7.,3.),(10.5,3.),(7.,8.),(10.5,8.),(14.5,5.5),(3.8,5.5),(15.5,2.5),(15.5,8.7)]
    for attempt in range(100):
        obstacles=[]
        for x,y in centers[:count]:
            obstacles.append(dict(type='box',kind='box',x=float(x+rng.uniform(-.45,.45)),y=float(y+rng.uniform(-.45,.45)),
                sx=float(rng.uniform(.65,.95)),sy=float(rng.uniform(1.1,1.6)),yaw=0.,height=1.0))
        start=[1.2,1.3,0.]
        if vary_start:
            start=list([(1.2,1.3,0.),(18.8,1.3,math.pi),(1.2,10.,0.),(18.8,10.,math.pi)][seed%4])
            start[2]+=float(rng.uniform(-math.pi/2,math.pi/2))
        # Two wall targets and one box target; mix all mounting directions.
        walls=[(float(rng.uniform(2,18)),.13,math.pi/2),(float(rng.uniform(2,18)),11.22,-math.pi/2),
               (.13,float(rng.uniform(2,9)),0.),(20.02,float(rng.uniform(2,9)),math.pi)]
        positions=[walls[int(i)] for i in rng.choice(4,2,replace=False)]
        o=obstacles[int(rng.integers(len(obstacles)))];face=int(rng.integers(4));a=face*math.pi/2
        d=(o['sx']/2 if face%2==0 else o['sy']/2)+.055
        positions.append((o['x']+d*math.cos(a),o['y']+d*math.sin(a),a))
        ids=rng.choice(1024,3,replace=False)
        targets=[dict(id=int(ident),x=x,y=y,yaw=a,size=float(rng.uniform(.16,.30))) for ident,(x,y,a) in zip(ids,positions)]
        cfg=dict(seed=int(seed),task='pdf_assignment_v1',difficulty=count,randomize=False,start=start,obstacles=obstacles,targets=targets)
        if valid(cfg):return cfg
    raise RuntimeError(f'Cannot create accessible assignment {seed}')


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',default='scenarios_assignment');a=p.parse_args()
    root=Path(a.output);manifest={}
    for split,start,n in [('train',710000,80),('validation',720000,24),('test',730000,48),('final_test',740000,48)]:
        dest=root/split;dest.mkdir(parents=True,exist_ok=True);manifest[split]=[]
        for i in range(n):
            seed=start+i;cfg=make_assignment(seed,[4,6,8][i%3],vary_start=True)
            (dest/f'{seed}.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False));manifest[split].append(seed)
    (root/'manifest.yaml').write_text(yaml.safe_dump(manifest))

if __name__=='__main__':main()
