import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import run_directed_integrated_screen as runner


def test_success_is_rejected_when_installed_candidate_profile_is_missing(tmp_path,monkeypatch):
    job=dict(output=str(tmp_path),floor_sensor_gate=True,runtime_configuration_gate=True,
             velocity_profile_gate=True,slam_transport_gate=True)
    result=dict(marker_directed_look=dict(version=1,target_frame='odom',camera_axis='optical_z',
        creates_recognition_or_approach_credit=False,ground_truth_inputs=False),
        near_pass_inspection=dict(maximum_sim_s=35.,maximum_looks=8,maximum_camera_range_m=1.2),
        marker_focus_profile=dict(desired_edge_pixels=36.,uncertain_view_penalty_s=8.),
        graph_landmark_update=dict(ground_truth_inputs=False))
    def collect(value):value.update(status='done',outcome=dict(success=True,reason='all_targets_found'))
    monkeypatch.setattr(runner,'collect',collect)
    (tmp_path/'report.json').write_text(json.dumps(dict(rows=[dict(result=result)])))
    runner.collect_screen(job);assert job['outcome']['success']
    del result['marker_directed_look']
    (tmp_path/'report.json').write_text(json.dumps(dict(rows=[dict(result=result)])))
    runner.collect_screen(job)
    assert not job['outcome']['success'] and job['raw_outcome_before_integrated_gates']['success']


def test_held_owner_with_actual_native_claim_cannot_be_replaced(tmp_path,monkeypatch):
    (tmp_path/'state.json').write_text(json.dumps(dict(status='held_for_directed_marker_fix',
        coordinator_unit='old-owner.service',jobs=[dict(status='prepared',unit='old-native.service',output=str(tmp_path/'case'))])))
    monkeypatch.setattr(runner,'service_active',lambda unit:False)
    monkeypatch.setattr(runner,'benchmark_output_live',lambda output:True)
    with pytest.raises(RuntimeError,match='native claim'):runner.validate_held_owner(tmp_path)


def test_second_case_reserves_slot_again_without_stopping_any_native(tmp_path,monkeypatch):
    parent=tmp_path/'parent';parent.mkdir()
    (parent/'state.json').write_text(json.dumps(dict(status='running',coordinator_unit='parent.service')))
    output=tmp_path/'screen';output.mkdir()
    jobs=[]
    for i in range(2):
        path=output/f'case_{i}';path.mkdir()
        jobs.append(dict(number=i,status='prepared',phase='diagnostic' if i==0 else 'train',
                         output=str(path),unit=f'native-{i}.service',domain=131+i,scenario=dict(seed=i)))
    (output/'state.json').write_text(json.dumps(dict(status='running',jobs=jobs,source_hashes={},driver_hashes={})))
    events=[];active={'parent.service':True};native_poll={};capacity_checks=[]
    monkeypatch.setattr(runner,'wait_for_dependency',lambda *a:None)
    monkeypatch.setattr(runner,'validate_held_owner',lambda *a:None)
    monkeypatch.setattr(runner,'source_hashes',lambda *a:{})
    monkeypatch.setattr(runner,'settle_claims',lambda *a:None)
    monkeypatch.setattr(runner,'measure',lambda *a:None)
    monkeypatch.setattr(runner,'service_active',lambda unit:active.get(unit,False))
    monkeypatch.setattr(runner,'benchmark_output_live',lambda output:False)
    monkeypatch.setattr(runner,'quality',lambda rows:dict(count=len(rows)))
    def resume(path):active['parent.service']=True;events.append(('resume',))
    monkeypatch.setattr(runner,'resume_cem',resume)
    def capacity():
        capacity_checks.append(1)
        # Each acquisition first observes two live existing missions, then one.
        return [100,101] if len(capacity_checks)%2 else [100]
    monkeypatch.setattr(runner,'native_processes',capacity)
    def pid(unit,directory):
        native_poll[unit]=native_poll.get(unit,0)+1
        if native_poll[unit]==1:return 200
        active[unit]=False;return None
    monkeypatch.setattr(runner,'native_pid',pid)
    def command(argv,**kw):
        if argv[:3]==['systemctl','--user','stop']:
            assert argv[3]=='parent.service';active[argv[3]]=False;events.append(('stop_dispatcher',))
        else:
            assert not active['parent.service'] and len(capacity_checks)%2==0
            unit=argv[argv.index('--unit')+1]+'.service';active[unit]=True
            events.append(('launch',unit))
        return SimpleNamespace(returncode=0,stdout='',stderr='')
    monkeypatch.setattr(runner.subprocess,'run',command)
    monkeypatch.setattr(runner.time,'sleep',lambda *a:None)
    def collect(job):job.update(status='done',outcome=dict(success=False));events.append(('collected',job['unit']))
    monkeypatch.setattr(runner,'collect_screen',collect)
    runner.run(parent,tmp_path/'dependency',tmp_path/'held',tmp_path/'workspace',output)
    assert len([e for e in events if e[0]=='launch'])==2
    assert len([e for e in events if e[0]=='stop_dispatcher'])==2
    assert len(capacity_checks)==4
    assert active['parent.service']
    assert json.loads((output/'state.json').read_text())['status']=='completed_integrated_screen'
