"""Balanced, sequential CPU-thread comparison on the same recorded scans.

This measures a SLAM component, not Gazebo throughput or mission success.
No native runtime settings or inputs are changed.
"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import resource
import subprocess
import time

import numpy as np

from native_tournament import native_processes


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def run(episode, executable, prefix, output):
    output.mkdir(parents=True,exist_ok=False)
    datasets=list(episode.glob('*/map.data'))
    if len(datasets)!=1:raise ValueError('Require one finalized sensor dataset')
    dataset=datasets[0];graph=dataset.with_suffix('.posegraph')
    results=[]
    sequence=['system',12,2,1,1,2,12,'system']
    for number,threads in enumerate(sequence):
        stem=output/f'{number:02d}_{threads}'
        env=dict(os.environ,ROS_DOMAIN_ID='134',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
        env.pop('OSP_REPLAY_SOLVER',None);env.pop('OSP_REPLAY_SOLVER_THREADS',None)
        if threads!='system':
            env.update(AMENT_PREFIX_PATH=str(prefix)+':'+env.get('AMENT_PREFIX_PATH','/opt/ros/jazzy'),
                       OSP_REPLAY_SOLVER='osp_solver_budget::CeresSolver',OSP_REPLAY_SOLVER_THREADS=str(threads))
        command=[str(executable),str(dataset),str(graph),str(stem),'150','baseline']
        before=resource.getrusage(resource.RUSAGE_CHILDREN);start=time.monotonic()
        native_before=native_processes();peak_threads=0;peak_rss=0
        with stem.with_suffix('.process.log').open('w') as stream:
            process=subprocess.Popen(command,stdout=stream,stderr=stream,env=env)
            while process.poll() is None:
                try:
                    status=dict(line.split(':',1) for line in Path(f'/proc/{process.pid}/status').read_text().splitlines() if ':' in line)
                    peak_threads=max(peak_threads,int(status['Threads']))
                    peak_rss=max(peak_rss,int(status.get('VmHWM','0 kB').split()[0]))
                except (OSError,KeyError,ValueError):pass
                time.sleep(.03)
        wall=time.monotonic()-start;after=resource.getrusage(resource.RUSAGE_CHILDREN)
        row=dict(number=number,threads=threads,exit_code=process.returncode,wall_seconds=wall,
                 cpu_seconds=(after.ru_utime+after.ru_stime-before.ru_utime-before.ru_stime),
                 voluntary_context_switches=after.ru_nvcsw-before.ru_nvcsw,
                 involuntary_context_switches=after.ru_nivcsw-before.ru_nivcsw,
                 peak_process_threads=peak_threads,peak_rss_KiB=peak_rss,
                 native_before=native_before,native_after=native_processes(),output=str(stem))
        results.append(row)
        (output/'progress.json').write_text(json.dumps(results,indent=2)+'\n')
        print(json.dumps(row),flush=True)
        if process.returncode:raise RuntimeError('Component arm failed; inspect retained process log')
    reference=None
    for row in results:
        with Path(row['output']+'.online.csv').open() as stream:records=list(csv.DictReader(stream))
        matrix=np.array([[float(r[k]) for k in ('original_node_id','stamp_sim_s','accepted','map_x','map_y','map_yaw','edges')] for r in records])
        if reference is None:reference=matrix
        row['scan_count']=len(records)
        row['same_scan_acceptance_and_edge_counts']=bool(matrix.shape==reference.shape and np.array_equal(matrix[:,[0,1,2,6]],reference[:,[0,1,2,6]]))
        row['maximum_pose_difference_from_first_system']=float(np.max(abs(matrix[:,3:6]-reference[:,3:6]))) if matrix.shape==reference.shape else None
    groups={str(k):dict(wall_median_s=float(np.median([r['wall_seconds'] for r in results if r['threads']==k])),
                       cpu_median_s=float(np.median([r['cpu_seconds'] for r in results if r['threads']==k]))) for k in ('system',12,2,1)}
    report=dict(evidence_type='recorded_SLAM_component_thread_budget_comparison',sequence=sequence,rows=results,groups=groups,
        input_hashes={str(p):digest(p) for p in [dataset,graph,executable,prefix/'lib/libosp_solver_budget.so']},
        native_runtime_changed=False,production_promoted=False,full_Gazebo_speedup_verified=False,
        limits=['Two repetitions per arm under changing host load; not a standalone stable throughput estimate.',
                'Recorded scans are a retained subset and do not include rendering,camera decoding,physics or navigation.',
                'Custom12 versus custom2/1 uses the same build; system uses the installed distribution build.',
                'Native Gazebo concurrency is preserved and each arm records actual native handles.'])
    (output/'evaluation.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('episode','executable','prefix','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();run(**{k:getattr(args,k).resolve() for k in ('episode','executable','prefix','output')})
