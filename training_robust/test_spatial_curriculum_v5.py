import json

import pytest

from spatial_native_curriculum_v5 import finish_phase,training_workspace


def config():
    return dict(workspace='/incumbent',baseline_workspace='/reference',
        runtime_alternatives=['/incumbent','/witness','/bounded_gain'])


def test_source_variants_are_explored_and_winning_source_follows_future_phases():
    cfg=config();state=dict(champion_workspace='/incumbent')
    assert [training_workspace(cfg,state,'train',str(i)) for i in range(4)]==[
        '/incumbent','/incumbent','/witness','/bounded_gain']
    state['champion_workspace']='/bounded_gain'
    assert [training_workspace(cfg,state,'train',str(i)) for i in range(4)]==[
        '/bounded_gain','/bounded_gain','/incumbent','/witness']
    for phase in ['pilot','validation','spatial_holdout','holdout']:
        assert training_workspace(cfg,state,phase,'candidate')=='/bounded_gain'
        assert training_workspace(cfg,state,phase,'baseline')=='/reference'
    state['champion_workspace']='/unfrozen'
    with pytest.raises(ValueError):training_workspace(cfg,state,'pilot','candidate')


def test_training_retains_runtime_of_winning_coefficients(tmp_path):
    jobs=[]
    for i,time in enumerate([400,350,200,250]):
        for shape in range(7):
            jobs.append(dict(generation=0,phase='train',role=str(i),
                workspace='/incumbent' if i<2 else '/bounded_gain',parameters=[1.,1.+i/10,1.,1.,1.],
                outcome=dict(success=True,seconds=time,targets=3,contacts=0)))
    state=dict(generation=0,phase='train',jobs=jobs,champion=[1]*5,mean=[0]*5,std=[.4]*5)
    finish_phase(tmp_path,config(),state)
    assert state['phase']=='pilot' and state['champion_workspace']=='/bounded_gain'
    assert state['champion']==[1.,1.2,1.,1.,1.]
    assert json.loads((tmp_path/'generation_0000_ranking.json').read_text())[0]['workspace']=='/bounded_gain'


def test_final_policy_artifact_points_to_audited_champion_runtime(tmp_path):
    state=dict(generation=0,phase='holdout',champion=[1]*5,champion_workspace='/bounded_gain',
        reserved_holdout_consumed=False,jobs=[dict(generation=0,phase='holdout',
            outcome=dict(success=True,seconds=200,targets=3,contacts=0)) for _ in range(40)])
    finish_phase(tmp_path,config(),state)
    policy=json.loads((tmp_path/'candidate_policy.json').read_text())
    assert policy['workspace']=='/bounded_gain'
    assert state['status']=='candidate_ready_for_completion_audit'


def test_launcher_executes_source_search_driver_not_older_fixed_runtime_driver(tmp_path,monkeypatch):
    import prepare_spatial_navigation_search as preparation
    parent=tmp_path/'parent';parent.mkdir()
    (parent/'state.json').write_text(json.dumps(dict(status='held_for_navigation_regression')))
    calls=[]
    monkeypatch.setattr(preparation.subprocess,'run',lambda command,**kwargs:calls.append(command))
    preparation.launch(parent,tmp_path/'successor')
    assert any(str(argument).endswith('/spatial_native_curriculum_v5.py') for argument in calls[0])
    assert not any(str(argument).endswith('/spatial_native_curriculum_v4.py') for argument in calls[0])
    assert json.loads((parent/'state.json').read_text())['status']=='superseded_by_spatial_cem_v5'


def shape_trials(phase,count,role='candidate'):
    return [dict(generation=0,phase=phase,role=role,scenario=dict(arena=dict(shape=shape)),
        outcome=dict(success=True,seconds=200,targets=3,contacts=0))
        for shape in ('circle','triangle','trapezoid','rectangle','l_shape','u_shape','rooms')
        for _ in range(count)]


def test_runtime_search_still_requires_every_shape_not_only_overall_success(tmp_path):
    current=shape_trials('validation',4)
    for j in current[:2]:j['outcome']['success']=False
    reference=shape_trials('validation',4,'baseline')
    for j in reference:j['outcome']['success']=False
    state=dict(generation=0,phase='validation',jobs=current+reference,champion=[1]*5,
        champion_workspace='/bounded_gain')
    finish_phase(tmp_path,config(),state)
    assert state['phase']=='train' and state['generation']==1
    assert not (tmp_path/'qualified_candidate.json').exists()


def test_successful_new_spaces_still_require_original_pdf_final_benchmark(tmp_path):
    state=dict(generation=0,phase='spatial_holdout',jobs=shape_trials('spatial_holdout',6),
        champion=[1]*5,champion_workspace='/bounded_gain',reserved_holdout_consumed=False)
    finish_phase(tmp_path,config(),state)
    assert state['phase']=='holdout' and not state['reserved_holdout_consumed']
    assert not (tmp_path/'candidate_policy.json').exists()
