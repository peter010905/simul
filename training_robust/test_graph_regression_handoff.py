import json
from types import SimpleNamespace

import pytest

import run_graph_landmark_regression as runner


def parent(tmp_path,status='running'):
    (tmp_path/'state.json').write_text(json.dumps(dict(status=status,coordinator_unit='osp-spatial-cem-v6.service')))
    return tmp_path


def test_unloaded_cem_uses_correct_v6_driver_when_diagnostic_hands_slot_back(tmp_path,monkeypatch):
    path=parent(tmp_path);active=iter([False,True]);commands=[]
    monkeypatch.setattr(runner,'service_active',lambda unit:next(active))
    monkeypatch.setattr(runner.subprocess,'run',lambda *args,**kwargs:SimpleNamespace(stdout='not-found\n',returncode=0))
    monkeypatch.setattr(runner,'start_unit',lambda unit,command,log:commands.append((unit,command,log)))
    runner.resume_cem(path)
    assert len(commands)==1 and commands[0][0]=='osp-spatial-cem-v6.service'
    assert str(runner.ROOT/'training_robust/spatial_native_curriculum_v6.py') in commands[0][1]
    assert not any('spatial_native_curriculum_v5.py' in value for value in commands[0][1])


def test_active_parent_is_not_stopped_or_started_again(tmp_path,monkeypatch):
    path=parent(tmp_path)
    monkeypatch.setattr(runner,'service_active',lambda unit:True)
    monkeypatch.setattr(runner.subprocess,'run',lambda *args,**kwargs:pytest.fail('No repeated service action'))
    monkeypatch.setattr(runner,'start_unit',lambda *args:pytest.fail('No duplicate start'))
    runner.resume_cem(path)


def test_terminal_or_repair_needed_campaign_state_is_not_overridden(tmp_path,monkeypatch):
    path=parent(tmp_path,'needs_infrastructure_repair')
    monkeypatch.setattr(runner,'start_unit',lambda *args:pytest.fail('Do not resume terminal campaign'))
    with pytest.raises(RuntimeError,match='Do not override'):runner.resume_cem(path)
