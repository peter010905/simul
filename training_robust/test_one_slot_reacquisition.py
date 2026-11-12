import json
from types import SimpleNamespace

import run_one_slot_reacquisition as runner


def test_resuming_unloaded_transient_dispatcher_recreates_same_frozen_driver(tmp_path,monkeypatch):
    (tmp_path/'state.json').write_text(json.dumps(dict(status='running',coordinator_unit='osp-spatial-cem-v5.service')))
    active=False;calls=[]
    monkeypatch.setattr(runner,'service_active',lambda unit:active)
    def execute(command,**kwargs):
        nonlocal active
        calls.append(command)
        if command[0]=='systemd-run':active=True
        return SimpleNamespace(returncode=0,stdout='not-found\n',stderr='')
    monkeypatch.setattr(runner.subprocess,'run',execute)
    runner.resume_parent(tmp_path)
    assert len(calls)==2 and calls[-1][0]=='systemd-run'
    assert any(str(arg).endswith('/spatial_native_curriculum_v5.py') for arg in calls[-1])
    assert json.loads((tmp_path/'state.json').read_text())['status']=='running'


def test_existing_live_dispatcher_is_never_duplicated(tmp_path,monkeypatch):
    (tmp_path/'state.json').write_text(json.dumps(dict(status='running',coordinator_unit='osp-spatial-cem-v5.service')))
    monkeypatch.setattr(runner,'service_active',lambda unit:True)
    monkeypatch.setattr(runner.subprocess,'run',lambda *args,**kwargs:(_ for _ in ()).throw(AssertionError('duplicate')))
    runner.resume_parent(tmp_path)
