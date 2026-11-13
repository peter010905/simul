from types import SimpleNamespace

import run_face_intent_regression_v2 as runner


def test_unloaded_transient_supervisor_is_recreated_with_original_driver(tmp_path,monkeypatch):
    active=[False];commands=[]
    monkeypatch.setattr(runner,'service_active',lambda unit:active[0])
    monkeypatch.setattr(runner.subprocess,'run',lambda *a,**kw:SimpleNamespace(returncode=5))
    def start(unit,command,log):commands.append(command);active[0]=True
    monkeypatch.setattr(runner,'start_unit',start)
    runner.restore_screen('osp-directed-integrated-v2.service',tmp_path)
    assert len(commands)==1 and commands[0][-1]==str(tmp_path)
    assert commands[0][-2].endswith('run_directed_integrated_screen_v2.py')


def test_already_active_original_supervisor_is_preserved(tmp_path,monkeypatch):
    monkeypatch.setattr(runner,'service_active',lambda unit:True)
    monkeypatch.setattr(runner.subprocess,'run',lambda *a,**kw:(_ for _ in ()).throw(AssertionError('unnecessary restart')))
    runner.restore_screen('osp-directed-integrated-v2.service',tmp_path)
