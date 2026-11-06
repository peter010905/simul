"""Hard-case diagnostics. Ground truth is used only in returned evaluator metrics."""
import argparse,json,math,time,hashlib
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np,yaml
from osp_mission.fast_sim import SearchEnv
from osp_mission.policy import load_policy
from osp_mission.inspection_policy import DEFAULT
from osp_mission.train import summary


def task(args):
 cfg,planner,w=args[:3];camera_hz=args[3] if len(args)>3 else None;inspection_frames=args[4] if len(args)>4 else 3;stress=cfg.get('sensor_stress',{});settings=dict(args[5]) if len(args)>5 else {}
 width=settings.pop('camera_width',960)
 collect_timing=settings.pop('collect_motion_timing',False)
 e=SearchEnv(cfg,seed=cfg['seed']+123456,planner=planner,camera_width=width,detection_dropout=stress.get('detection_dropout',.08),camera_hz=camera_hz,inspection_frames=inspection_frames,**settings)
 e.speed=min(e.speed,stress.get('speed_limit',.26))
 r=e.run(w);trace=np.array(e.trace);trace=trace[trace[:,3]<=600];nearest=[float(np.hypot(trace[:,0]-t['x'],trace[:,1]-t['y']).min()) for t in e.cfg['targets']]
 r.update(mount_profile=cfg['mount_profile'],actual_obstacles=len(cfg['obstacles']),speed=e.speed,
   dropout=e.detection_dropout,approached=sum(d<=2 for d in nearest),travel_glances=getattr(e,'travel_glances',0),
   missing=[dict(id=t['id'],mount=t['mount'],size=t['size'],nearest=nearest[i],confirmations=e.confirmations.get(t['id'],0)) for i,t in enumerate(cfg['targets']) if t['id'] not in e.found])
 if collect_timing:r.update(observation_alignment_seconds=e.observation_alignment_seconds,observation_sweep_seconds=e.observation_sweep_seconds,avoidable_alignment_radians=e.avoidable_alignment_radians)
 return r


def summarize(rows):
 s=summary(rows)
 s['by_mount']={k:summary([r for r in rows if r['mount_profile']==k]) for k in sorted({r['mount_profile'] for r in rows})}
 s['mean_approached']=float(np.mean([r['approached'] for r in rows]));return s

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--split',default='validation');p.add_argument('--planner',default='legacy');p.add_argument('--weights',default='');p.add_argument('--output',required=True);p.add_argument('--workers',type=int,default=4);p.add_argument('--camera-hz',type=float,default=None);p.add_argument('--inspection-frames',type=int,default=3);a=p.parse_args()
 cfgs=[yaml.safe_load(p.read_text()) for p in sorted((Path('scenarios_robust')/a.split).glob('*.yaml'))]
 if not cfgs:raise ValueError('No scenarios')
 w=load_policy('src/osp_mission/config/assignment_policy.json') if a.planner=='legacy' else DEFAULT
 if a.weights:w=np.array(json.loads(Path(a.weights).read_text())['parameters'])
 start=time.monotonic()
 source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in Path('src/osp_mission/osp_mission').glob('*.py')}
 with ProcessPoolExecutor(max_workers=a.workers) as pool:rows=list(pool.map(task,[(c,a.planner,w,a.camera_hz,a.inspection_frames) for c in cfgs]))
 report=dict(source_hashes=source_hashes,environment='2d_surrogate',planner=a.planner,camera_width=960,camera_hz=a.camera_hz,inspection_frames=a.inspection_frames,deadline=600,parameters=w.tolist(),scenario_hash=hashlib.sha256(json.dumps(cfgs,sort_keys=True).encode()).hexdigest(),summary=summarize(rows),rows=rows,wall_seconds=time.monotonic()-start)
 Path(a.output).write_text(json.dumps(report,indent=2));print(json.dumps(report['summary']),flush=True)
