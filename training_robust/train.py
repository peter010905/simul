"""Worst-mount-aware CEM; held-out test is never loaded here."""
import argparse,json,hashlib,shutil,time,importlib
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np,yaml
from evaluate import task,summarize
from osp_mission.inspection_policy import DEFAULT
from osp_mission.curriculum import LayoutCurriculum, validate_split_separation


def key(s):
    return (-s['collisions'],min(v['success_rate'] for v in s['by_mount'].values()),s['success_rate'],s['mean_found'],-s['mean_seconds'])

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',default='training_robust/surface_cem');p.add_argument('--generations',type=int,default=6);p.add_argument('--workers',type=int,default=4);p.add_argument('--planner',default='surface_endpoint');p.add_argument('--camera-hz',type=float,default=5.);p.add_argument('--initial-policy',default='');p.add_argument('--train-dir',default='scenarios_robust/train');p.add_argument('--validation-dir',default='scenarios_robust/validation');p.add_argument('--seed',type=int,default=2026110501);p.add_argument('--resume',action='store_true');p.add_argument('--generation-limit',type=int,default=0);a=p.parse_args()
 if a.generation_limit<0:raise ValueError('Generation limit must be nonnegative')
 out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
 if (out/'configuration.json').exists() and not a.resume:raise ValueError('Use a new output directory or --resume')
 cfgs={s:[yaml.safe_load(p.read_text()) for p in sorted(Path(directory).glob('*.yaml'))] for s,directory in [('train',a.train_dir),('validation',a.validation_dir)]}
 if not cfgs['validation']:raise ValueError('Validation split is empty')
 validate_split_separation(cfgs['train'],cfgs['validation'])
 curriculum=LayoutCurriculum(cfgs['train'],a.seed+1)
 scenario_hashes={s:hashlib.sha256(json.dumps(c,sort_keys=True).encode()).hexdigest() for s,c in cfgs.items()}
 if not a.resume:
  snapshot=out/'source_snapshot';snapshot.mkdir(exist_ok=True)
  package=Path(importlib.import_module('osp_mission').__file__).parent
  for source in package.glob('*.py'):shutil.copy2(source,snapshot/source.name)
  shutil.copy2(__file__,snapshot/'training_driver.py');shutil.copy2(Path(__file__).parent/'evaluate.py',snapshot/'evaluate.py')
  (out/'configuration.json').write_text(json.dumps(dict(generations=a.generations,seed=a.seed,layout_seed=a.seed+1,sampling='stratified_shuffled_without_replacement',train_dir=a.train_dir,validation_dir=a.validation_dir,layout_counts={s:len(c) for s,c in cfgs.items()},planner=a.planner,camera_hz=a.camera_hz,initial_policy=a.initial_policy,population=12,batch=20,camera_width=960,deadline=600,source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.glob('*.py')},scenario_hashes=scenario_hashes,validation_elites=3),indent=2))
 rng=np.random.default_rng(a.seed);champion=np.array(json.loads(Path(a.initial_policy).read_text())['parameters']) if a.initial_policy else DEFAULT.copy();mean=np.log(champion);std=np.full(5,.9);history=[];start=time.monotonic()
 start_generation=0;best=None;elapsed_before=0.;validation_elites=3
 if a.resume:
  configuration=json.loads((out/'configuration.json').read_text())
  validation_elites=configuration.get('validation_elites',1)
  for field,value in [('planner',a.planner),('camera_hz',a.camera_hz),('seed',a.seed),('generations',a.generations),('scenario_hashes',scenario_hashes)]:
   if configuration[field]!=value:raise ValueError(f'Resume configuration mismatch: {field}')
  for name in ['fast_sim','directional_policy','surface_endpoint_policy','surface_policy','inspection_policy','face_policy','policy','scenarios','robust_scenarios']:
   if name+'.py' not in configuration['source_hashes']:continue
   module=importlib.import_module('osp_mission.'+name)
   if hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest()!=configuration['source_hashes'][name+'.py']:
    raise ValueError(f'Resume source mismatch: {name}; use the recorded snapshot')
  if hashlib.sha256((Path(__file__).parent/'evaluate.py').read_bytes()).hexdigest()!=configuration['source_hashes']['evaluate.py']:raise ValueError('Resume evaluator mismatch')
  with np.load(out/'checkpoint.npz') as checkpoint:
   start_generation=int(checkpoint['generation'])+1;mean=checkpoint['mean'].copy();std=checkpoint['std'].copy();champion=checkpoint['champion'].copy()
   rng.bit_generator.state=json.loads(str(checkpoint['rng_state']));curriculum.restore(json.loads(str(checkpoint['curriculum_state'])))
  previous=json.loads((out/'history.json').read_text());history=previous[:start_generation]
  if len(history)!=start_generation:raise ValueError('Checkpoint lacks completed history')
  if len(previous)!=len(history):(out/f'interrupted_history_{start_generation}.json').write_text(json.dumps(previous,indent=2))
  best=history[-1]['best'];elapsed_before=history[-1]['wall_seconds']
  recovery=out/'recoveries.json';recoveries=json.loads(recovery.read_text()) if recovery.exists() else []
  recoveries.append(dict(next_generation=start_generation,driver_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),loaded_package=str(Path(importlib.import_module('osp_mission').__file__).parent),original_sources_verified=True,time_unix=time.time()))
  recovery.write_text(json.dumps(recoveries,indent=2));print(json.dumps(dict(resumed_generation=start_generation,training_episodes=start_generation*240)),flush=True)
 def save():
  tmp=out/'best_policy.tmp';tmp.write_text(json.dumps(dict(planner=a.planner,parameters=champion.tolist(),validation=best,camera_hz=a.camera_hz,training_episodes=len(history)*240),indent=2));tmp.replace(out/'best_policy.json')
 with ProcessPoolExecutor(max_workers=a.workers) as pool:
  if best is None:best=summarize(list(pool.map(task,[(c,a.planner,champion,a.camera_hz) for c in cfgs['validation']])));save()
  end_generation=min(a.generations,start_generation+a.generation_limit) if a.generation_limit else a.generations
  for g in range(start_generation,end_generation):
   batch=curriculum.next_batch()
   (out/'current_batch.json').write_text(json.dumps(dict(generation=g,seeds=[c['seed'] for c in batch],sampling='stratified_shuffled_without_replacement'),indent=2))
   population=np.exp(np.clip(rng.normal(mean,std,(12,5)),-3,3));population[:,0]=1.;population[0]=champion
   rows=list(pool.map(task,[(c,a.planner,w,a.camera_hz) for w in population for c in batch]))
   scores=[];training_statistics=[]
   for i in range(12):
    r=rows[i*20:(i+1)*20];s=summarize(r)
    training_statistics.append(s)
    worst=min(x['success_rate'] for x in s['by_mount'].values())
    scores.append(5000*worst+5000*s['success_rate']+100*s['mean_found']-.1*s['mean_seconds']-100000*s['collisions'])
   elite=np.log(population[np.argsort(scores)[-3:]]);mean=.3*mean+.7*elite.mean(axis=0);std=np.maximum(.25,.4*std+.6*elite.std(axis=0))
   ranked=sorted(range(12),key=lambda index:-scores[index])[:validation_elites]
   validations=[]
   for index in ranked:
    candidate=population[index]
    stats=best if np.array_equal(candidate,champion) else summarize(list(pool.map(task,[(c,a.planner,candidate,a.camera_hz) for c in cfgs['validation']])))
    validations.append(dict(index=index,parameters=candidate.tolist(),validation=stats))
   selected=max(validations,key=lambda row:key(row['validation']))
   candidate=population[selected['index']];stats=selected['validation']
   if key(stats)>key(best):champion=candidate.copy();best=stats
   history.append(dict(generation=g,batch_seeds=[c['seed'] for c in batch],candidate_parameters=candidate.tolist(),validation=stats,best=best,validation_candidates=validations,training_candidates=[dict(parameters=w.tolist(),score=float(score),summary=summary) for w,score,summary in zip(population,scores,training_statistics)],training_episodes=(g+1)*240,wall_seconds=elapsed_before+time.monotonic()-start))
   (out/'history.json').write_text(json.dumps(history,indent=2));save()
   with (out/'checkpoint.tmp').open('wb') as stream:np.savez(stream,mean=mean,std=std,champion=champion,generation=g,rng_state=json.dumps(rng.bit_generator.state),curriculum_state=json.dumps(curriculum.state()))
   (out/'checkpoint.tmp').replace(out/'checkpoint.npz')
   print(json.dumps(dict(generation=g,best_success=best['success_rate'],worst_mount=min(v['success_rate'] for v in best['by_mount'].values()),training_episodes=(g+1)*240,wall_seconds=elapsed_before+time.monotonic()-start)),flush=True)
 if len(history)==a.generations:
  (out/'completed.json').write_text(json.dumps(dict(generations=a.generations,training_episodes=a.generations*240,test_evaluated=False),indent=2))
 else:
  (out/f'chunk_finished_{len(history):03d}.json').write_text(json.dumps(dict(next_generation=len(history),training_episodes=len(history)*240,training_complete=False,reason='generation_limit'),indent=2))
  print(json.dumps(dict(chunk_finished=True,next_generation=len(history),training_complete=False)),flush=True)
