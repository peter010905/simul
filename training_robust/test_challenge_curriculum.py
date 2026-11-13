"""The new topology curriculum keeps the independent physical completion gates."""
import copy
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest

import challenge_native_curriculum as runner
from probe_encoder_delta_profile import EXPECTED as ENCODER
from probe_velocity_profile import EXPECTED as LEGACY


def test_generation_uses_all_three_shapes_hidden_targets_and_fresh_scenes(tmp_path):
    from osp_mission.challenge_scenarios import make_challenge_scene
    (tmp_path/'scenarios').mkdir()
    cfg=dict(seed=1192500000,workspace='/frozen_candidate',baseline_workspace='/frozen_baseline',run_id='test',
             baseline_weights=[1.,2.,.5,.8,.6])
    state=dict(phase='train',generation=0,jobs=[],champion=cfg['baseline_weights'],mean=np.log(cfg['baseline_weights']).tolist(),std=[0,.4,.4,.4,.4])
    runner.plan(tmp_path,cfg,state)
    jobs=state['jobs']
    assert len(jobs)==12
    assert {j['scenario']['arena']['shape'] for j in jobs}=={'c_shape','u_shape','zigzag'}
    assert len({j['scenario']['physical_sha256'] for j in jobs})==12
    assert all(j['scenario']['mount_profile']=='hidden_obstacles' for j in jobs)
    assert all(j['workspace']=='/frozen_candidate' for j in jobs)
    assert all(j['scenario']['requested_obstacles']==j['scenario']['actual_obstacles'] for j in jobs)
    state.update(phase='pilot',planned=False)
    runner.plan(tmp_path,cfg,state)
    pairs=state['jobs'][12:]
    assert len(pairs)==6
    for i in range(0,6,2):
        assert pairs[i]['scenario']==pairs[i+1]['scenario']
        assert pairs[i]['workspace']=='/frozen_candidate' and pairs[i+1]['workspace']=='/frozen_baseline'


def test_motion_profile_and_physical_gates_are_not_relaxed():
    modern=dict(complete=True,parameters=copy.deepcopy(ENCODER));old=dict(complete=True,parameters=copy.deepcopy(LEGACY))
    _,a=runner.motion_profile(dict(role='candidate'));_,b=runner.motion_profile(dict(role='baseline'))
    assert a(modern) and not a(old) and b(old) and not b(modern)
    assert not runner.strict_credit_gate(dict(all_three_physical_credits_verified=True,rows=[]))
    baseline=dict(episodes=3,success_rate=1.,contacts=0,par2_seconds=400.)
    assert runner.materially_better(dict(baseline,par2_seconds=300.),baseline,3,1.)
    assert not runner.materially_better(dict(baseline,contacts=1,par2_seconds=300.),baseline,3,1.)


def test_qualification_requires_three_families_and_fresh_holdout(tmp_path):
    jobs=[]
    for shape in ['c_shape','u_shape','zigzag']:
        for i in range(4):
            for role,seconds in [('candidate',250.),('baseline',400.)]:
                jobs.append(dict(generation=0,phase='validation',role=role,scenario=dict(arena=dict(shape=shape)),
                                 outcome=dict(success=True,seconds=seconds,contacts=0,targets=3)))
    state=dict(generation=0,phase='validation',jobs=jobs,champion=[1]*5,champion_workspace='/frozen')
    runner.finish_phase(tmp_path,{},state)
    assert state['phase']=='spatial_holdout'
    # One entirely failed family cannot be hidden by the other two families.
    state=dict(generation=0,phase='validation',jobs=copy.deepcopy(jobs),champion=[1]*5,champion_workspace='/frozen')
    for j in state['jobs']:
        if j['role']=='candidate' and j['scenario']['arena']['shape']=='zigzag':j['outcome']['success']=False
    runner.finish_phase(tmp_path,{},state)
    assert state['phase']=='train' and state['generation']==1


def test_handoff_stops_only_dispatcher_and_preserves_native_runs(tmp_path,monkeypatch):
    import launch_challenge_curriculum as launcher
    root=tmp_path/'native';parent=root/'parent';regression=root/'regression';destination=root/'challenge';held=root/'directed_integrated_screen_v1'
    for p in [parent,regression,destination,held]:p.mkdir(parents=True)
    active_native='osp-existing-native.service'
    (parent/'state.json').write_text(json.dumps(dict(status='running',coordinator_unit='osp-old-cem.service',
        jobs=[dict(status='running',unit=active_native,output=str(root/'live'),scenario=dict(seed=1))])))
    (regression/'state.json').write_text(json.dumps(dict(coordinator_unit='osp-finished-diagnostic.service')))
    (held/'state.json').write_text(json.dumps(dict(status='held_for_face_source_training_upgrade',coordinator_unit='osp-held.service')))
    (destination/'state.json').write_text(json.dumps(dict(status='running',jobs=[])))
    monkeypatch.setattr(launcher,'service_active',lambda unit:unit in ['osp-old-cem.service',active_native])
    commands=[];started=[];drained=[]
    monkeypatch.setattr(launcher.subprocess,'run',lambda command,**kw:commands.append(command))
    monkeypatch.setattr(launcher,'settle_claims',lambda *a:None)
    monkeypatch.setattr(launcher,'prime_parent_inputs',lambda *a:None)
    monkeypatch.setattr(launcher,'start_unit',lambda *a:started.append(a))
    monkeypatch.setattr(launcher,'drain',lambda *a:drained.append(a))
    launcher.launch(parent,regression,destination)
    assert commands==[['systemctl','--user','stop','osp-old-cem.service']]
    assert len(started)==3 and started[0][0]==launcher.UNIT
    assert drained==[(parent,destination)]
    old=json.loads((parent/'state.json').read_text())
    assert old['jobs'][0]['status']=='running' and old['jobs'][0]['unit']==active_native
    assert old['original_prepared_jobs_retained']


@pytest.mark.parametrize('shape',['c_shape','u_shape','zigzag'])
def test_native_world_keeps_concave_floor_at_world_zero_and_collision_walls(tmp_path,shape):
    import importlib.util
    import xml.etree.ElementTree as ET
    import yaml
    from osp_mission.challenge_scenarios import make_challenge_scene
    root=Path(__file__).resolve().parents[1]
    source=root/'policy_releases/2026-11-13_face_intent_v2/runtime/src/osp_mission/osp_mission/world.py'
    spec=importlib.util.spec_from_file_location('native_challenge_world',source)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    cfg=make_challenge_scene(1192300000,8,shape=shape)
    scene=tmp_path/'scene.yaml';scene.write_text(yaml.safe_dump(cfg))
    sdf,_=module.generate(scene,tmp_path/'world',recording=True)
    world=ET.parse(sdf).find('world')
    walls=[m for m in world.findall('model') if m.attrib['name'].startswith('arena_wall')]
    assert len(walls)==len(cfg['arena']['polygon'])
    assert all(m.find('link/collision') is not None for m in walls)
    floor=world.find("model[@name='floor']");visual=floor.find('link/visual')
    offset=np.array([float(v) for v in floor.find('pose').text.split()[:3]])
    offset+=np.array([float(v) for v in visual.find('pose').text.split()[:3]])
    mesh=Path(visual.find('geometry/mesh/uri').text)
    vertices=np.array([[float(v) for v in line.split()[1:]] for line in mesh.read_text().splitlines() if line.startswith('v ')])
    np.testing.assert_allclose(vertices[:,:2]+offset[:2],cfg['arena']['polygon'])
    np.testing.assert_allclose(vertices[:,2]+offset[2],0.)
