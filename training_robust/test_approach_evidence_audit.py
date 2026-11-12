import json

from audit_approach_evidence import audit,physical_at_stamp


def test_an_earlier_close_pass_does_not_validate_a_later_far_credit():
    trace=[[0.,1.,0.],[.2,1.02,0.],[.4,1.04,0.],[10.,2.,0.],[10.2,2.02,0.]]
    assert physical_at_stamp(trace,[0,0],.1)['verified']
    assert not physical_at_stamp(trace,[0,0],10.1)['verified']


def test_interpolation_alone_cannot_credit_boundary_without_speed_margin():
    result=physical_at_stamp([[0,1.49,0],[.2,1.49,0]],[0,0],.1)
    assert result['actual_distance_m']<1.5
    assert result['conservative_distance_upper_m']>1.5
    assert not result['verified']


def test_gaps_jumps_and_extrapolation_remain_unverified():
    assert not physical_at_stamp([[0,1,0],[1,1,0]],[0,0],.5)['verified']
    assert not physical_at_stamp([[0,1,0],[.2,2,0]],[0,0],.1)['verified']
    assert not physical_at_stamp([[0,1,0],[.2,1,0]],[0,0],.3)['verified']
    assert not physical_at_stamp([[.2,1,0],[0,1,0]],[0,0],.1)['verified']


def test_exact_physical_sample_does_not_need_interpolation_margin():
    result=physical_at_stamp([[0,1.5,0],[.2,1.5,0]],[0,0],.2)
    assert result['verified'] and result['actual_distance_m']==1.5


def test_finer_evidence_cannot_be_hidden_by_a_coarse_historical_pass(tmp_path):
    header='sim_seconds,world_x,world_y,world_yaw\n'
    (tmp_path/'ground_truth_trajectory.csv').write_text(header+'0,1,0,0\n0.2,1,0,0\n')
    (tmp_path/'evaluator_gt_fine_fixture.csv').write_text(header+'0,3,0,0\n0.2,3,0,0\n')
    (tmp_path/'evaluation.json').write_text(json.dumps(dict(approach=dict(
        approach_evidence={'1':dict(stamp=.1)}))))
    (tmp_path/'marker_events.jsonl').write_text(json.dumps(dict(event='id_confirmed',id=1))+'\n')
    scenario=tmp_path/'scene.yaml';scenario.write_text('targets: [{id: 1, x: 0, y: 0}]\n')
    result=audit(tmp_path,scenario)['rows'][0]
    assert result['map_pose_credit']['trajectory_source']=='fine_evaluator_trace'
    assert result['map_pose_credit']['actual_distance_m']==3
    assert not result['physical_credit_verified']
