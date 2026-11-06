"""Fixed-policy replay: examine face rewards and missed-target opportunities."""

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from functools import partial
import hashlib
import json
import math
from pathlib import Path
import shutil

import numpy as np
import yaml
from scipy.ndimage import label,find_objects

from osp_mission.fast_sim import SearchEnv
from osp_mission.edge_policy import options,choose
from osp_mission.plane_views import symmetric_grid
from osp_mission.face_mass import weights as face_weights,reweight


def score(candidates,parameters):
    if not candidates:return np.zeros(0)
    values=np.asarray([c['features'] for c in candidates])
    return ((parameters[0]*values[:,0]+parameters[1]*values[:,1])/
            (parameters[2]*values[:,2]/.26+parameters[3]*values[:,3]/.5+5.)-parameters[4]*values[:,4]*.1)


def potential(env,candidate,target):
    dx=target['x']-candidate['x'];dy=target['y']-candidate['y'];distance=math.hypot(dx,dy)
    if distance<.25:return False
    front=(-dx*math.cos(target['yaw'])-dy*math.sin(target['yaw']))/distance
    if front<.45 or distance>.95*env.camera_focal*target['size']*front/18:return False
    relative=(math.atan2(dy,dx)-candidate['yaw']+math.pi)%(2*math.pi)-math.pi
    # An optimistic geometrical opportunity, not a decoder-confirmation claim.
    if abs(candidate['scan_angle'])<2*math.pi-1e-9:
        phase=(relative+1.25/2)%(2*math.pi)
        if phase>candidate['scan_angle']+1.25:return False
    ds=np.arange(.1,max(.11,distance-.20),env.res*.35)
    xs=((candidate['x']+ds*dx/distance)/env.res).astype(int).clip(0,env.map.shape[1]-1)
    ys=((candidate['y']+ds*dy/distance)/env.res).astype(int).clip(0,env.map.shape[0]-1)
    return not env.truth[ys,xs].any()


def task(arguments):
    reference,out=arguments;seed=reference['seed']
    cfg=yaml.safe_load(Path(f'scenarios_robust/v2/train/{seed}.yaml').read_text())
    parameters=json.loads(Path('training_robust/symmetric_candidate.json').read_text())['parameters']
    env=SearchEnv(cfg,seed=seed+123456,planner='symmetric_edge',camera_width=960,camera_hz=5.,
                  detection_dropout=cfg['sensor_stress']['detection_dropout'],resolution=.05,
                  lidar_samples=640,lidar_body_frame=True,raycast_backend='native',continuous_shortcuts=True,
                  frame_quarter_turns=(seed//20)%4)
    env.speed=min(env.speed,cfg['sensor_stress']['speed_limit'])
    env.options=partial(options,point_sampler=symmetric_grid,collect_faces=True)
    decisions=[]
    def record(candidates,parameters):
        selected=choose(candidates,parameters)
        if selected is None:return None
        # Select the real action first. Nothing from this analysis changes it.
        raw=score(candidates,parameters)
        capped=reweight(candidates,face_weights(env.map,env.res,1.5),env.res)
        alternate=score(capped,parameters)
        groups,_=label(env.map>50,structure=np.ones((3,3)))
        walls={i for i,s in enumerate(find_objects(groups),start=1) if s is not None and max(t.stop-t.start for t in s)*env.res>=6.}
        faces=selected['visible_faces'];ids=groups[faces[:,1],faces[:,2]]
        wall_fraction=float(np.isin(ids,list(walls)).mean()) if len(ids) else 0.
        order=np.argsort(-raw,kind='stable');ranks=np.empty(len(order),dtype=int);ranks[order]=np.arange(1,len(order)+1)
        alt_order=np.argsort(-alternate,kind='stable');alt_ranks=np.empty(len(order),dtype=int);alt_ranks[alt_order]=np.arange(1,len(order)+1)
        opportunities=[]
        for target in env.cfg['targets']:
            if target['id'] in env.found:continue
            available=[i for i,c in enumerate(candidates) if potential(env,c,target)]
            opportunities.append(dict(id=target['id'],available=len(available),
                best_raw_rank=min((int(ranks[i]) for i in available),default=None),
                best_capped_rank=min((int(alt_ranks[i]) for i in available),default=None),
                selected_potential=potential(env,selected,target)))
        decisions.append(dict(time=env.elapsed,candidates=len(candidates),selected_wall_fraction=wall_fraction,
                              selected_surface=float(selected['features'][0]),
                              selected_capped_surface=float(capped[int(np.argmax(raw))]['features'][0]),targets=opportunities))
        return selected
    env.choose=record;result=env.run(parameters)
    for key,value in result.items():assert value==reference[key],(seed,key,value,reference[key])
    missing={t['id'] for t in env.cfg['targets']} - env.found
    for decision in decisions:decision['targets']=[t for t in decision['targets'] if t['id'] in missing]
    report=dict(seed=seed,exact_replay=True,result=result,missing=sorted(missing),decisions=decisions,
                diagnostic_only_ground_truth=True)
    path=Path(out)/f'{seed}.json';path.write_text(json.dumps(report,indent=2))
    return report


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    parser.add_argument('--workers',type=int,default=10);args=parser.parse_args()
    out=Path(args.output);out.mkdir(exist_ok=False)
    snapshot=out/'source_snapshot';package=snapshot/'osp_mission';package.mkdir(parents=True)
    for source in Path('src/osp_mission/osp_mission').iterdir():
        if source.suffix in ('.py','.cpp','.so'):shutil.copy2(source,package/source.name)
    shutil.copy2(__file__,snapshot/'driver.py')
    references=[]
    for name in ['geometry_view_screen','symmetric_expansion']:
        references.extend(r for r in json.loads(Path(f'training_robust/{name}/symmetric_edge.json').read_text())['rows'] if not r['success'])
    (out/'configuration.json').write_text(json.dumps(dict(split='v2/train',seeds=[r['seed'] for r in references],
        test_loaded=False,validation_loaded=False,source_hashes={str(p.relative_to(snapshot)):hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.rglob('*') if p.is_file()}),indent=2))
    rows=[]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        work={pool.submit(task,(reference,str(out))):reference['seed'] for reference in references}
        for future in as_completed(work):
            row=future.result();rows.append(row);print(json.dumps(dict(seed=row['seed'],completed=len(rows),total=len(references))),flush=True)
    moments=[(d,t) for r in rows for d in r['decisions'] for t in d['targets'] if t['available']]
    summary=dict(episodes=len(rows),exact_replays=True,missed_markers=sum(len(r['missing']) for r in rows),
        missed_with_any_candidate=sum(any(t['available'] for d in r['decisions'] for t in d['targets'] if t['id']==ident) for r in rows for ident in r['missing']),
        opportunity_decisions=len(moments),selected_wall_majority=sum(d['selected_wall_fraction']>.5 for d,t in moments),
        raw_top1=sum(t['best_raw_rank']==1 for d,t in moments),capped_top1=sum(t['best_capped_rank']==1 for d,t in moments),
        raw_top5=sum(t['best_raw_rank']<=5 for d,t in moments),capped_top5=sum(t['best_capped_rank']<=5 for d,t in moments),
        note='Counterfactual scores on a fixed trajectory; not candidate-policy success, and potential visibility does not guarantee arrival or three decoded frames')
    (out/'completed.json').write_text(json.dumps(dict(summary=summary,rows=rows),indent=2));print(json.dumps(summary),flush=True)


if __name__=='__main__':main()
