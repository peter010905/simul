import json
from pathlib import Path

from native_tournament import add_jobs, ranking


def test_success_precedes_speed_and_failures_stay_in_denominator():
    jobs=[dict(phase='screen',candidate='safe',status='done',outcome=dict(success=True,seconds=550,targets=3,approached=3)),
          dict(phase='screen',candidate='fast',status='done',outcome=dict(success=True,seconds=100,targets=3,approached=3)),
          dict(phase='screen',candidate='fast',status='done',outcome=dict(success=False,infrastructure_failure=True))]
    rows=ranking(jobs,['fast','safe'],'screen')
    assert rows[0]['candidate']=='safe'
    assert rows[1]['success_rate']==.5


def test_prepared_jobs_keep_real_benchmark_protocol(tmp_path):
    policy=tmp_path/'policy.json';policy.write_text('{"planner":"edge","parameters":[1,1,1,1,1]}')
    config=dict(run_id='test',candidates={'a':dict(policy=str(policy),workspace='/frozen/workspace')})
    state={'jobs':[]};scene=dict(seed=17,file='/private/evaluator/17.yaml',sha256='frozen')
    add_jobs(tmp_path,state,config,'screen',['a'],[scene])
    job=state['jobs'][0];manifest=json.loads((Path(job['output'])/'manifest.json').read_text())
    assert manifest['duration']==600 and manifest['approach_radius_m']==1.5
    assert manifest['record_video'] and manifest['scenarios']==[scene]
    assert job['status']=='prepared' and 'outcome' not in job


def test_development_search_finishes_without_reading_reserved_holdout(tmp_path, monkeypatch):
    import native_tournament
    config = dict(candidates={'control': dict(workspace='/frozen/control', policy='/frozen/policy')},
                  holdout_enabled=False)
    state = dict(status='running', phase='validation', jobs=[dict(
        status='done', phase='validation', candidate='control',
        outcome=dict(success=True, targets=3, approached=3, seconds=150))])
    (tmp_path/'configuration.json').write_text(json.dumps(config))
    (tmp_path/'state.json').write_text(json.dumps(state))
    monkeypatch.setattr(native_tournament, 'check_sources', lambda _: None)
    monkeypatch.setattr('sys.argv', ['native_tournament.py', str(tmp_path)])
    native_tournament.main()
    result = json.loads((tmp_path/'selected_candidate.json').read_text())
    assert result['reserved_holdout_used'] is False
    assert result['candidate']['success_rate'] == 1.
    final = json.loads((tmp_path/'state.json').read_text())
    assert final['status'] == 'completed_development_search'
    assert final['goal_verified'] is False
