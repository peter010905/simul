from probe_heading_prior import gate


def test_frozen_expected_profile_must_be_observed_from_complete_native_response():
    good=dict(complete=True,parameters={'slam_toolbox':{'angle_variance_penalty':.05,'minimum_angle_penalty':.5}})
    assert gate(good)
    assert not gate(dict(good,complete=False))
    assert not gate(dict(complete=True,parameters={'slam_toolbox':{'angle_variance_penalty':1.,'minimum_angle_penalty':.9}}))
    assert not gate(dict(complete=True,parameters={'slam_toolbox':{'angle_variance_penalty':.05}}))
