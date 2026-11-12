"""Start the frozen spatial campaign even after the old transient unit unloads.

Recovery entry point for a prepared, never-launched campaign. Runtime sources,
maps and candidate parameters are not changed.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time

from continual_native_curriculum import collect, digest, save, source_hashes
from native_tournament import service_active

ROOT = Path(__file__).resolve().parents[1]
UNIT = 'osp-spatial-cem-v1.service'


def stop_previous():
    result = subprocess.run(['systemctl','--user','show','osp-fresh-map-cem-v2.service',
                             '-p','LoadState','--value'],text=True,capture_output=True)
    if result.stdout.strip() not in ('', 'not-found'):
        subprocess.run(['systemctl','--user','stop','osp-fresh-map-cem-v2.service'],check=True)


def start(parent, destination):
    config = json.loads((destination/'configuration.json').read_text())
    for path, expected in config['source_hashes'].items():
        if source_hashes(path) != expected:
            raise RuntimeError(f'Frozen runtime changed: {path}')
    for path, expected in config['driver_hashes'].items():
        if digest(Path(path)) != expected:
            raise RuntimeError(f'Frozen driver changed: {path}')
    stop_previous()
    previous = json.loads((parent/'state.json').read_text())
    waiting = [j for j in previous['jobs'] if j['status'] == 'running']
    previous.update(status='superseded_by_spatial_cem_v1',successor=str(destination))
    for job in waiting:
        if not service_active(job['unit']):
            collect(job)
    save(parent/'state.json',previous)
    save(parent/'successor_transition.json',dict(status='draining_previous_native_jobs',
        destination=str(destination),units=[j['unit'] for j in waiting],original_results_preserved=True,
        supervision_recovery='Old transient coordinator already stopped/unloaded; no native run interrupted'))
    try:
        subprocess.run(['systemd-run','--user',f'--unit={UNIT}',f'--property=WorkingDirectory={ROOT}',
            '--property=MemoryMax=512M','--property=Restart=no',
            f'--property=StandardOutput=append:{destination/"coordinator.log"}',
            f'--property=StandardError=append:{destination/"coordinator.log"}',
            '/usr/bin/python3','-u',str(ROOT/'training_robust/spatial_native_curriculum.py'),str(destination)],check=True)
        if not service_active(UNIT):
            raise RuntimeError('Spatial coordinator did not start')
    except BaseException as exc:
        save(destination/'handoff_error.json',dict(error=repr(exc),native_runs_preserved=True,
                                                  retry_entrypoint=str(Path(__file__).resolve())))
        raise
    while any(service_active(j['unit']) for j in waiting):
        time.sleep(5)
    for job in waiting:
        collect(job)
    save(parent/'state.json',previous)
    save(parent/'learning_log.json',[dict(number=j['number'],phase=j['phase'],scenario=j['scenario'],
        parameters=j['parameters'],outcome=j['outcome']) for j in previous['jobs'] if j['status']=='done'])
    save(parent/'successor_transition.json',dict(status='successor_active',service=UNIT,
        destination=str(destination),original_results_preserved=True,goal_verified=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--parent',type=Path,required=True)
    parser.add_argument('--destination',type=Path,required=True)
    args = parser.parse_args()
    start(args.parent.resolve(),args.destination.resolve())
