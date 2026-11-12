"""Read-only native progress and completed-episode analysis beside frozen CEM.

This observer never changes a score, policy, scenario, source, service or job
state. Fine ground truth is used only by the evaluator-side physical audit.
"""
import argparse
from collections import Counter,defaultdict
import datetime
import json
from pathlib import Path
import shlex
import subprocess
import time

from analyze_navigation_regression import summarize
from audit_approach_evidence import audit
from continual_native_curriculum import save

ROOT=Path(__file__).resolve().parents[1]


def native_pid(unit,output):
    result=subprocess.run(['systemctl','--user','show',unit,'-p','MainPID','-p','ActiveState'],
        capture_output=True,text=True,timeout=5)
    values=dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)
    pid=int(values.get('MainPID','0'))
    try:argv=Path(f'/proc/{pid}/cmdline').read_bytes().decode().split('\0')
    except OSError:argv=[]
    return pid if values.get('ActiveState')=='active' and any(
        v.endswith('/gazebo_benchmark.py') for v in argv) and str(output) in argv else None


def observe(root):
    workers={};analyses={};attempted=set()
    while True:
        state=json.loads((root/'state.json').read_text());live=[];errors=[]
        for job in state['jobs']:
            output=Path(job['output']);episode=output/'episodes'/str(job['scenario']['seed'])
            if job['status']=='running':
                pid=native_pid(job['unit'],output)
                live.append(dict(number=job['number'],unit=job['unit'],verified_native_pid=pid,
                    workspace=job['workspace'],shape=job['scenario'].get('arena',{}).get('shape')))
                if pid and episode.exists() and job['number'] not in attempted and len([
                        p for p in workers.values() if p.poll() is None])<2:
                    target=episode/f'evaluator_gt_fine_{time.time_ns()}.csv'
                    command='source /opt/ros/jazzy/setup.bash\nexec '+shlex.join([
                        '/usr/bin/python3',str(ROOT/'training_robust/record_evaluator_truth.py'),
                        '--domain',str(job['domain']),'--native-pid',str(pid),
                        '--native-output',str(output),'--output',str(target)])
                    with (output/'evaluator_truth_recorder.log').open('a') as stream:
                        workers[job['number']]=subprocess.Popen(['/bin/bash','-c',command],stdout=stream,stderr=stream)
                    attempted.add(job['number'])
            if job['status']!='done' or job['number'] in analyses:continue
            if job['number'] in workers and workers[job['number']].poll() is None:continue
            try:
                row=summarize(job)
                row.update(number=job['number'],generation=job['generation'],phase=job['phase'],role=job['role'],
                    shape=job['scenario'].get('arena',{}).get('shape'),
                    arena=job['scenario'].get('arena'),obstacles=job['scenario'].get('actual_obstacles'))
                row['physical_credit_audit']=audit(episode,Path(job['scenario']['file']))
                row['log_files']=dict(marker_events=str(episode/'marker_events.jsonl'),
                    video=str(episode/'video/mission.mp4'),evaluation=str(episode/'evaluation.json'))
                save(output/'observer_analysis.json',row);analyses[job['number']]=row
            except (OSError,ValueError,KeyError,TypeError) as exc:
                errors.append(dict(number=job['number'],observation_error=repr(exc)))
        groups=defaultdict(list)
        for row in analyses.values():groups[(row['phase'],Path(row['workspace']).parent.name,row['shape'])].append(row)
        summary=[]
        for (phase,source,shape),rows in groups.items():
            outcomes=[r['outcome'] for r in rows]
            summary.append(dict(phase=phase,source=source,shape=shape,episodes=len(rows),
                successes=sum(r['success'] for r in outcomes),
                infrastructure_failures=sum(r['infrastructure_failure'] for r in outcomes),
                par2_seconds=sum(r['seconds'] if r['success'] else 1200 for r in outcomes)/len(rows),
                physical_credits_verified=sum(r['physical_credit_audit']['all_three_physical_credits_verified'] for r in rows)))
        report=dict(captured_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
            purpose='Read-only ongoing CEM/evaluator observatory; never a robot policy input',
            campaign_status=state['status'],generation=state['generation'],phase=state['phase'],
            job_status_counts=dict(Counter(j['status'] for j in state['jobs'])),live=live,
            completed_analyses=len(analyses),groups=summary,observation_errors=errors,
            marker_log_files=[row['log_files']['marker_events'] for row in analyses.values()],
            classification='progress' if analyses else 'verified_wait' if any(r['verified_native_pid'] for r in live) else 'observation_pending',
            limits=['Training rows are descriptive training results, not an independent population success probability',
                'Different source variants also use different coefficient candidates and scenes; pooled results do not isolate a code change',
                'Returned path includes necessary corridor transit; do not equate it with wasted distance'])
        save(root/'observer_progress.json',report)
        if state['status'] not in ('running',) and not live and not any(p.poll() is None for p in workers.values()):return
        time.sleep(10)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path);observe(parser.parse_args().root.resolve())
