"""Supervise a finite sequence of frozen native regression campaigns."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

from native_tournament import native_processes, service_active


def save(path, data):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data,indent=2)+'\n');tmp.replace(path)


def hashes(root):
    return {str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in root.rglob('*') if p.is_file() and not {'__pycache__','.pytest_cache'}.intersection(p.parts)}


def run(root):
    path=root/'sequence.json';state=json.loads(path.read_text())
    while state['status']=='running':
        for job in state['jobs']:
            if job['status']!='running' or service_active(job['unit']):continue
            report=Path(job['output'])/'report.json'
            if not report.exists():raise RuntimeError(f'{job["name"]} exited without report')
            data=json.loads(report.read_text())
            if len(data['rows'])!=len(json.loads((Path(job['output'])/'manifest.json').read_text())['scenarios']):
                raise RuntimeError(f'{job["name"]} exited with incomplete campaign')
            if data['infrastructure_failures']:
                raise RuntimeError(f'{job["name"]} has infrastructure failures')
            subprocess.run([sys.executable,str(Path(__file__).with_name('audit_native_campaign.py')),job['output']],check=True,capture_output=True,text=True)
            job.update(status='done',successes=data['successful_missions'],episodes=data['attempted'])
            print(json.dumps(dict(name=job['name'],successes=job['successes'],episodes=job['episodes'])),flush=True)
            save(path,state)
        if all(j['status']=='done' for j in state['jobs']):
            state['status']='completed';save(path,state);break
        if shutil.disk_usage(root).free<8*1024**3:raise RuntimeError('Less than8GiB free')
        statuses={j['name']:j['status'] for j in state['jobs']}
        for job in state['jobs']:
            if job['status']!='prepared' or any(statuses[n]!='done' for n in job['after']):continue
            if max(len(native_processes()),sum(j['status']=='running' for j in state['jobs']))>=2:break
            source=Path(job['workspace'])/'src/osp_mission'
            if hashes(source)!=job['source_hashes']:raise RuntimeError('Prepared source changed: '+job['name'])
            for file,digest in job['artifact_hashes'].items():
                if hashlib.sha256(Path(file).read_bytes()).hexdigest()!=digest:raise RuntimeError('Prepared inputs changed')
            cmd=[sys.executable,str(Path(__file__).with_name('run_native_service.py')),'--unit',job['unit'].removesuffix('.service'),
                 '--workspace',job['workspace'],'--output',job['output'],'--',sys.executable,
                 str(Path(__file__).with_name('gazebo_benchmark.py')),'--output',job['output'],'--run-prepared','--domain',str(job['domain']),'--record-video']
            subprocess.run(cmd,check=True,capture_output=True,text=True)
            job['status']='running';save(path,state)
        time.sleep(5.)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path);args=parser.parse_args();root=args.root.resolve()
    try:run(root)
    except Exception as exc:
        path=root/'sequence.json';state=json.loads(path.read_text());state.update(status='needs_attention',error=repr(exc),updated_utc=datetime.datetime.now(datetime.timezone.utc).isoformat());save(path,state);raise
