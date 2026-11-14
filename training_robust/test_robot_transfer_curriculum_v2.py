import copy
import json
import math
import numpy as np
import yaml
import robot_transfer_curriculum_v2 as runner
from probe_transfer_input_v2 import gate
from probe_encoder_delta_profile import EXPECTED


def test_training_and_pairs_keep_profiles_frozen_without_ground_truth_policy(tmp_path):
    (tmp_path/'scenarios').mkdir()
    cfg=dict(seed=1194000000,workspace='/candidate',baseline_workspace='/baseline',run_id='test',baseline_weights=[1.,2.,.5,.8,.6])
    state=dict(phase='train',generation=0,jobs=[],champion=cfg['baseline_weights'],mean=np.log(cfg['baseline_weights']).tolist(),std=[0,.3,.3,.3,.3])
    runner.plan(tmp_path,cfg,state)
    assert len(state['jobs'])==12
    assert {j['stress_profile'] for j in state['jobs']}=={'none','mild','moderate'}
    assert len({j['scenario']['physical_sha256'] for j in state['jobs']})==12
    for job in state['jobs']:
        scene=yaml.safe_load(open(job['scenario']['file']))
        assert -math.pi<=scene['start'][2]<=math.pi
        policy=json.load(open(job['output']+'/policy.json'))
        assert not {'targets','start','scenario','stress_seed'} & policy.keys()
    state['phase']='pilot';runner.plan(tmp_path,cfg,state)
    for a,b in zip(state['jobs'][12::2],state['jobs'][13::2]):
        assert a['scenario']==b['scenario']
        assert a['stress_profile']==b['stress_profile'] and a['stress_seed']==b['stress_seed']
        assert a['workspace']!=b['workspace']


def test_stress_requires_actual_scan_and_wheel_observations():
    report=dict(scan=dict(profile='mild',received=5,acquisition_stamp_preserved=True,ground_truth_input=False),
                wheel=dict(transfer_profile='mild',accepted=50,ground_truth_input=False))
    assert gate(report,'mild')
    assert not gate(report,'moderate')
    assert not gate(dict(scan=report['scan']),'mild')
    report['scan']['received']=0
    assert not gate(report,'mild')


def test_equal_motion_profiles_and_strict_contact_gate():
    measured=dict(complete=True,parameters=copy.deepcopy(EXPECTED))
    for role in ['candidate','baseline','0']:
        assert runner.motion_profile(dict(role=role))[1](measured)
    baseline=dict(episodes=12,success_rate=1.,contacts=0,par2_seconds=500.)
    assert not runner.materially_better(dict(baseline,contacts=1,par2_seconds=300.),baseline,12)
    assert not runner.strict_credit_gate(dict(all_three_physical_credits_verified=True,rows=[]))
