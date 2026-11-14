import json
from pathlib import Path
from types import SimpleNamespace
import publish_training_progress as publisher


def test_snapshot_separates_infrastructure_and_preserves_unrelated_readme(tmp_path,monkeypatch):
    campaign=tmp_path/'campaign';campaign.mkdir();(tmp_path/'repository_snapshot').mkdir()
    (tmp_path/'README.md').write_text('Keep this original text.\n')
    (tmp_path/'repository_snapshot/status.json').write_text('{"original_result":42}')
    jobs=[]
    for number,(success,infra) in enumerate([(True,False),(False,False),(False,True)]):
        jobs.append(dict(number=number,status='done',phase='train',role='0',stress_profile='mild',
            scenario=dict(arena=dict(shape='u_shape')),outcome=dict(success=success,infrastructure_failure=infra,contacts=0)))
    (campaign/'state.json').write_text(json.dumps(dict(status='running',generation=0,phase='train',jobs=jobs,coordinator_unit='osp-test.service')))
    monkeypatch.setattr(publisher.subprocess,'run',lambda *a,**k:SimpleNamespace(returncode=0))
    a=publisher.snapshot(tmp_path,tmp_path,campaign)
    assert a['recorded_completed']==3 and a['valid_completed']==2 and a['successes']==1 and a['infrastructure_failures']==1
    assert a['hardware_validated'] is False and a['production_promoted'] is False
    assert json.loads((tmp_path/'repository_snapshot/status.json').read_text())['original_result']==42
    publisher.snapshot(tmp_path,tmp_path,campaign)
    readme=(tmp_path/'README.md').read_text()
    assert readme.startswith('Keep this original text.') and readme.count(publisher.START)==1
