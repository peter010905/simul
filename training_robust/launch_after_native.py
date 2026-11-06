"""Launch one frozen prepared campaign after a specific native job exits."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('campaign',type=Path)
    args=parser.parse_args()
    root=args.campaign.resolve();file=root/'queued_launch.json'
    info=json.loads(file.read_text())
    def save(status,**extra):
        info.update(status=status,updated_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),**extra)
        file.write_text(json.dumps(info,indent=2)+'\n')
        print(json.dumps(dict(status=status,**extra)),flush=True)
    try:
        # Identify the existing process by its start time as well as PID.
        while True:
            try:
                text=Path(f'/proc/{info["wait_pid"]}/stat').read_text()
                fields=text[text.rfind(')')+2:].split()
                if fields[19]!=info['wait_pid_start_ticks'] or fields[0]=='Z':break
            except FileNotFoundError:break
            time.sleep(5.)
        prior=Path(info['wait_campaign'])
        manifest=json.loads((prior/'manifest.json').read_text())
        report=json.loads((prior/'report.json').read_text())
        if len(report['rows'])!=len(manifest['scenarios']):
            raise RuntimeError('Prior native campaign ended without all planned results; inspect it before continuing')
        source=Path(info['workspace'])/'src/osp_mission'
        hashes=json.loads((root/info['source_hashes_file']).read_text())
        actual={str(p.relative_to(source)):hashlib.sha256(p.read_bytes()).hexdigest()
                for p in source.rglob('*') if p.is_file() and not {'__pycache__','.pytest_cache'}.intersection(p.parts)}
        if actual!=hashes:raise RuntimeError('Queued candidate source changed; refusing to launch a different policy')
        for name in ('policy','manifest'):
            if hashlib.sha256((root/f'{name}.json').read_bytes()).hexdigest()!=info[f'{name}_sha256']:
                raise RuntimeError(f'Queued {name} changed')
        subprocess.run([sys.executable,str(Path(__file__).with_name('audit_native_campaign.py')),str(prior)],check=True)
        save('starting_native_campaign')
        result=subprocess.run(['bash','/home/peter/simul/run_gazebo_guarded.sh',sys.executable,
            '/home/peter/simul/training_robust/gazebo_benchmark.py','--output',str(root),
            '--run-prepared','--domain',str(info['domain']),'--record-video'],cwd=info['workspace'])
        save('native_campaign_process_exited',returncode=result.returncode)
        return result.returncode
    except Exception as exc:
        save('queue_failed',error=str(exc))
        return 1


if __name__=='__main__':
    raise SystemExit(main())
