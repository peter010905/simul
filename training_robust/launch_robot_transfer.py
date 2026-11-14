"""Prepare a new immutable campaign; never rewrite earlier queued runtimes."""
import argparse
import copy
import json
from pathlib import Path
import numpy as np

from challenge_native_curriculum import digest, save, source_hashes
from launch_spatial_curriculum import audit_geometry
from launch_spatial_cem_v6 import start_unit
from native_tournament import service_active
import robot_transfer_curriculum as successor

ROOT=Path(__file__).resolve().parents[1]
UNIT='osp-robot-transfer-cem-v1.service'


def prepare(parent, workspace, baseline, destination):
    if destination.exists():raise FileExistsError('Preserve existing campaign')
    prior=json.loads((parent/'configuration.json').read_text())
    prior_state=json.loads((parent/'state.json').read_text())
    if service_active(prior_state['coordinator_unit']):
        raise RuntimeError('Previous coordinator must be inactive; running native jobs are preserved')
    destination.mkdir();(destination/'scenarios').mkdir()
    config=copy.deepcopy(prior)
    config.update(run_id='nov14-transfer1',seed=1194000000,workspace=str(workspace),
        baseline_workspace=str(baseline),runtime_alternatives=[str(workspace)],domains=[151,152],
        parent_campaign=str(parent),production_promoted=False,hardware_validated=False,
        primary_baseline='Previous face-intent exploration with identical sensor stress/transport fixes',
        purpose='Open-path and repeated-viewpoint robustness under synthetic lidar/encoder stress',
        synthetic_profiles=['none','mild','moderate'],real_robot_calibration=False)
    config['source_hashes']={str(p):source_hashes(p) for p in (workspace,baseline)}
    files=set(config['driver_hashes'])|{str(Path(__file__).resolve()),str(ROOT/'training_robust/robot_transfer_curriculum.py'),
        str(ROOT/'training_robust/probe_transfer_input.py')}
    config['driver_hashes']={name:digest(Path(name)) for name in files}
    weights=prior_state['champion']
    config['baseline_weights']=weights
    save(destination/'configuration.json',config)
    state=dict(status='running',generation=0,phase='train',planned=False,jobs=[],champion=weights,
        champion_workspace=str(workspace),mean=np.log(weights).tolist(),std=[0.,.3,.3,.3,.3],
        reserved_holdout_consumed=prior_state.get('reserved_holdout_consumed',False),spatial_holdout_consumed=False,
        coordinator_unit=UNIT,source_fixed_during_campaign=True)
    successor.plan(destination,config,state)
    used={j.get('scenario',{}).get('seed') for p in parent.parent.glob('*/state.json') if p.parent!=destination
          for j in json.loads(p.read_text()).get('jobs',[])}
    if any(j['scenario']['seed'] in used for j in state['jobs']):raise ValueError('Previously reserved seed')
    save(destination/'state.json',state)
    return audit_geometry(destination)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('parent','workspace','baseline','destination'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--start',action='store_true');a=p.parse_args()
    paths=[getattr(a,k).resolve() for k in ('parent','workspace','baseline','destination')]
    print(json.dumps(prepare(*paths)))
    if a.start:
        start_unit(UNIT,['/usr/bin/python3','-u',str(ROOT/'training_robust/robot_transfer_curriculum.py'),str(paths[-1])],paths[-1]/'coordinator.log')
