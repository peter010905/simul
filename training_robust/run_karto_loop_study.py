"""Sequential component ablations across recorded geometries, never native trials."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

from evaluate_karto_replay import evaluate


def save(path,value):
    temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(value,indent=2)+'\n');temporary.replace(path)


def run(campaign,executable,prefix,output):
    output.mkdir(parents=True,exist_ok=False)
    original=json.loads((campaign/'state.json').read_text())
    state=dict(status='running',native_mission=False,policy_ground_truth_inputs=False,jobs=[],
        maximum_simultaneous_components=1,original_native_results_modified=False,
        source_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            [Path(__file__),Path(__file__).with_name('evaluate_karto_replay.py'),executable,prefix/'lib/libosp_solver_budget.so']})
    # Start with the observed false closure, then include all seven geometries,
    # an earlier different L failure and an additional successful room layout.
    for index in (11,0,1,2,3,4,5,6,13):
        job=original['jobs'][index]
        if job['status']!='done':raise ValueError('Require finalized native recordings')
        episode=Path(job['output'])/'episodes'/str(job['scenario']['seed'])
        datasets=list(episode.glob('*/map.data'))
        if len(datasets)!=1:raise ValueError('Require one immutable native dataset')
        directory=output/f'case_{index}';directory.mkdir()
        state['jobs'].append(dict(number=index,shape=job['scenario']['arena']['shape'],episode=str(episode),
            dataset=str(datasets[0]),output=str(directory),status='prepared',arms=[]))
    save(output/'state.json',state)
    for job in state['jobs']:
        job['status']='running';save(output/'state.json',state)
        dataset=Path(job['dataset']);directory=Path(job['output'])
        for variant in ('baseline','fine55','strict_loop'):
            if any(hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h for p,h in state['source_hashes'].items()):
                raise RuntimeError('Frozen component study changed')
            env=dict(os.environ,ROS_DOMAIN_ID='135',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',
                AMENT_PREFIX_PATH=str(prefix)+':'+os.environ.get('AMENT_PREFIX_PATH','/opt/ros/jazzy'),
                OSP_REPLAY_SOLVER='osp_solver_budget::CeresSolver',OSP_REPLAY_SOLVER_THREADS='2')
            started=time.monotonic()
            with (directory/(variant+'.process.log')).open('w') as stream:
                process=subprocess.run([str(executable),str(dataset),str(dataset.with_suffix('.posegraph')),
                    str(directory/variant),'1000000',variant],stdout=stream,stderr=stream,env=env)
            job['arms'].append(dict(variant=variant,exit_code=process.returncode,wall_seconds=time.monotonic()-started))
            save(output/'state.json',state)
            if process.returncode:raise RuntimeError('Component replay failed; preserve its evidence')
            report=evaluate(Path(job['episode']),directory);save(directory/'evaluation.json',report)
            print(json.dumps(dict(number=job['number'],variant=variant,metrics=next(r for r in report['arms'] if r['name']==variant))),flush=True)
        job['status']='done';save(output/'state.json',state)
    state['status']='completed';save(output/'state.json',state)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('campaign','executable','prefix','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();run(**{k:getattr(args,k).resolve() for k in ('campaign','executable','prefix','output')})
