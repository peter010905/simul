"""Versioned observer handoff; preserve native processes and frozen policy source."""
import argparse
import copy
import json
from pathlib import Path
import subprocess
from challenge_native_curriculum import digest, save
from launch_spatial_cem_v6 import start_unit
from native_tournament import service_active


def handoff(parent, destination):
    destination.mkdir(parents=True,exist_ok=False)
    (destination/'scenarios').mkdir()
    prior=json.loads((parent/'state.json').read_text())
    if service_active(prior['coordinator_unit']):
        subprocess.run(['systemctl','--user','stop',prior['coordinator_unit']],check=True)
    # Reload after dispatcher exit; accepted native units are independently owned.
    state=json.loads((parent/'state.json').read_text())
    config=json.loads((parent/'configuration.json').read_text())
    save(destination/'inherited_state.json',state)
    state=copy.deepcopy(state);state['coordinator_unit']='osp-robot-transfer-cem-v2.service'
    config.update(run_id='nov14-transfer2',seed=1194100000,parent_campaign=str(parent),
        observer_repair='Use executor bound to explicit ROS context; no policy/sensor change')
    here=Path(__file__).resolve().parent
    for path in [Path(__file__).resolve(),here/'robot_transfer_curriculum_v2.py',here/'probe_transfer_input_v2.py']:
        config['driver_hashes'][str(path)]=digest(path)
    save(destination/'configuration.json',config);save(destination/'state.json',state)
    prior.update(status='superseded_observer_context_repair',successor=str(destination),
        native_runs_preserved=True,prepared_artifacts_preserved=True)
    save(parent/'state.json',prior)
    start_unit(state['coordinator_unit'],['/usr/bin/python3','-u',str(here/'robot_transfer_curriculum_v2.py'),str(destination)],destination/'coordinator.log')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('parent',type=Path);p.add_argument('destination',type=Path);a=p.parse_args()
    handoff(a.parent.resolve(),a.destination.resolve())
