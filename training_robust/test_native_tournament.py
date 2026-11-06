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
