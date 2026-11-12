from spatial_native_curriculum_v4 import finish_phase

SHAPES=('circle','triangle','trapezoid','rectangle','l_shape','u_shape','rooms')


def jobs(phase,count):
    return [dict(generation=0,phase=phase,role='candidate',scenario=dict(arena=dict(shape=shape)),
        outcome=dict(success=True,targets=3,approached=3,contacts=0,seconds=200.,infrastructure_failure=False))
        for shape in SHAPES for _ in range(count)]


def test_v4_rejects_two_failures_hidden_in_one_shape(tmp_path):
    current=jobs('validation',4)
    for j in current[:2]:j['outcome']['success']=False
    current += [dict(j,role='baseline',outcome=dict(j['outcome'],success=False)) for j in jobs('validation',4)]
    state=dict(generation=0,phase='validation',jobs=current,champion=[1]*5)
    finish_phase(tmp_path,{},state)
    assert state['phase']=='train' and state['generation']==1
    assert not (tmp_path/'qualified_candidate.json').exists()


def test_v4_fresh_shapes_cannot_replace_original_pdf_final(tmp_path):
    state=dict(generation=0,phase='spatial_holdout',jobs=jobs('spatial_holdout',6),
        champion=[1]*5,reserved_holdout_consumed=False)
    finish_phase(tmp_path,{},state)
    assert state['phase']=='holdout' and not state['reserved_holdout_consumed']
    assert not (tmp_path/'candidate_policy.json').exists()
