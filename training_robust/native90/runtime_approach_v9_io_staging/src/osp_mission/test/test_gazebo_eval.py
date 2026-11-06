"""Evaluator failures must not launch an orphaned simulation."""
import json

from osp_mission import gazebo_eval


def test_ros_initialization_failure_is_recorded_before_launch(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))

    def fail_init(**kwargs):
        raise RuntimeError('DDS unavailable')

    def unexpected_launch(*args, **kwargs):
        raise AssertionError('Simulation launched before evaluator was ready')

    monkeypatch.setattr(gazebo_eval.rclpy, 'init', fail_init)
    monkeypatch.setattr(gazebo_eval.subprocess, 'Popen', unexpected_launch)
    result = gazebo_eval.one('scenario.yaml', 'policy.json', tmp_path)
    assert result['success'] is False
    assert result['reason'] == 'evaluator_startup_failure'
    assert json.loads((tmp_path / 'evaluation.json').read_text()) == result
