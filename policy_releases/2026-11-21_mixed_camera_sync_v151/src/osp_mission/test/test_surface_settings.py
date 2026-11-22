"""Malformed candidate settings must not silently change the mission policy."""

import json

import numpy as np
import pytest

from osp_mission.surface_endpoint_policy import load_configuration


def test_settings_default_is_backward_compatible_and_new_candidate_loads(tmp_path):
    path=tmp_path/'policy.json'
    payload=dict(planner='surface_endpoint',parameters=[1.,2.,1.,.3,1.])
    path.write_text(json.dumps(payload))
    weights,settings=load_configuration(path)
    np.testing.assert_array_equal(weights,payload['parameters'])
    assert settings==dict(view_standoff=2.,deadline_aware=False)
    payload['settings']=dict(view_standoff=2.4,deadline_aware=True)
    path.write_text(json.dumps(payload))
    assert load_configuration(path)[1]==payload['settings']


@pytest.mark.parametrize('settings',[
    dict(view_standoff=float('nan')),dict(view_standoff=0),dict(view_standoff=True),
    dict(view_standoff='2.4'),dict(deadline_aware='false'),None,[]])
def test_invalid_settings_are_rejected(tmp_path,settings):
    path=tmp_path/'policy.json'
    path.write_text(json.dumps(dict(planner='surface_endpoint',parameters=[1.,2.,1.,.3,1.],settings=settings)))
    with pytest.raises(ValueError):load_configuration(path)
