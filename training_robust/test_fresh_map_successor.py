import json

import pytest
import yaml

from prepare_fresh_map_successor import prepare, tf_decision


def fixture(tmp_path):
    original = tmp_path/'original'
    original.mkdir()
    baseline = tmp_path/'baseline'
    (baseline/'src/osp_mission').mkdir(parents=True)
    (baseline/'src/osp_mission/reference.py').write_text('reference = True\n')
    perception = tmp_path/'perception'
    (perception/'src/osp_mission/config').mkdir(parents=True)
    (perception/'src/osp_mission/test').mkdir()
    (perception/'src/osp_mission/config/slam.yaml').write_text(yaml.safe_dump(
        dict(slam_toolbox=dict(ros__parameters=dict(restamp_tf=False)))))
    driver = tmp_path/'frozen_driver.py'
    driver.write_text('driver = True\n')
    (original/'configuration.json').write_text(json.dumps(dict(
        run_id='old', seed=1000, workspace='/old', baseline_workspace=str(baseline),
        baseline_weights=[1.]*5, driver_hashes={str(driver): 'old_digest'})))
    (original/'state.json').write_text(json.dumps(dict(champion=[1.,2.,3.,4.,5.],
        mean=[0.,.1,.2,.3,.4], std=[.4]*5, reserved_holdout_consumed=True)))
    return original, perception


def test_successor_preserves_learning_but_starts_new_unique_map_sequence(tmp_path):
    original, perception = fixture(tmp_path)
    destination = tmp_path/'successor'
    workspace = prepare(original, perception, destination, dict(enabled=True))
    config = json.loads((destination/'configuration.json').read_text())
    state = json.loads((destination/'state.json').read_text())
    assert config['seed'] == 1123200000 and config['run_id'] == 'nov12-v2'
    assert config['workspace'] == str(workspace)
    assert state['champion'] == [1.,2.,3.,4.,5.] and state['mean'] == [0.,.1,.2,.3,.4]
    assert state['jobs'] == [] and state['generation'] == 0
    assert state['reserved_holdout_consumed'] is True
    source_config = yaml.safe_load((perception/'src/osp_mission/config/slam.yaml').read_text())
    new_config = yaml.safe_load((workspace/'src/osp_mission/config/slam.yaml').read_text())
    assert source_config['slam_toolbox']['ros__parameters']['restamp_tf'] is False
    assert new_config['slam_toolbox']['ros__parameters']['restamp_tf'] is True
    with pytest.raises(FileExistsError):
        prepare(original, perception, destination, dict(enabled=False))


def tf_fixture(tmp_path):
    regression = tmp_path/'tf'
    regression.mkdir()
    jobs, originals = [], []
    for index in range(2):
        old = tmp_path/f'old{index}'
        new = tmp_path/f'new{index}'
        for output in (old, new):
            (output/f'episodes/{index}').mkdir(parents=True)
        (old/f'episodes/{index}/launch.log').write_text(('Unable to transform robot pose into global plan\'s frame\n')*10)
        (new/f'episodes/{index}/launch.log').write_text('')
        (new/f'episodes/{index}/evaluation.json').write_text(json.dumps(dict(
            ground_truth_audit=dict(found_ids=[1], expected_ids=[1,2,3], position_errors_m={'1': .1}))))
        baseline = dict(success=False, seconds=600., targets=1, contacts=0, infrastructure_failure=False)
        outcome = dict(baseline, success=index == 0, seconds=200. if index == 0 else 600.)
        jobs.append(dict(scenario=dict(seed=index), output=str(new), baseline=baseline, outcome=outcome))
        originals.append(dict(scenario=dict(seed=index), output=str(old)))
    (regression/'regression_state.json').write_text(json.dumps(dict(
        status='completed_regression_requires_fresh_map_validation', jobs=jobs)))
    return regression, dict(jobs=originals), jobs


def test_tf_selection_requires_native_time_gain_and_frame_error_reduction(tmp_path):
    regression, original, jobs = tf_fixture(tmp_path)
    assert tf_decision(regression, original)['enabled'] is True
    (tmp_path/'new1/episodes/1/launch.log').write_text(("Unable to transform robot pose into global plan's frame\n")*8)
    assert tf_decision(regression, original)['enabled'] is False


def test_tf_selection_rejects_bad_landmarks_and_retains_perception_if_missing(tmp_path):
    regression, original, jobs = tf_fixture(tmp_path)
    path = tmp_path/'new1/episodes/1/evaluation.json'
    path.write_text(json.dumps(dict(ground_truth_audit=dict(found_ids=[999], expected_ids=[1,2,3]))))
    assert tf_decision(regression, original)['enabled'] is False
    assert tf_decision(tmp_path/'missing', original)['enabled'] is False
