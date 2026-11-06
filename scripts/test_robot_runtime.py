"""Regression checks for the hardware adapter without starting robot nodes."""
import ast
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import unittest

from prepare_robot_runtime import hardware_mission


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'training_robust/native90/runtime_fix_v13_observation_staging/src/osp_mission'


class HardwareRuntimeTest(unittest.TestCase):
    def test_observation_calls_use_ros_time(self):
        adapted = ast.parse(hardware_mission((SOURCE / 'osp_mission/mission.py').read_text()))
        expected = {
            'self.marker_focus.observe': (2, 1),
            'self.marker_focus.request_interrupt': (2, 1),
            'self.marker_focus.choose': (-1, 1),
            'self.approach.choose': (-1, 1),
            'self.approach.observe_pose': (-1, 1),
        }
        clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_800_000_000_000_000_000))
        fake = SimpleNamespace(get_clock=lambda: clock, now=lambda: 12345.)
        for name, (index, expected_count) in expected.items():
            calls = [node for node in ast.walk(adapted) if isinstance(node, ast.Call)
                     and ast.unparse(node.func) == name]
            self.assertEqual(len(calls), expected_count)
            for call in calls:
                expression = ast.Expression(body=call.args[index])
                self.assertEqual(eval(compile(expression, '<clock>', 'eval'), {'self': fake}), 1_800_000_000.)
        now_method = next(node for node in ast.walk(adapted)
                          if isinstance(node, ast.FunctionDef) and node.name == 'now')
        self.assertIn('time.monotonic()', ast.unparse(now_method))

    def test_real_epoch_pose_is_credited(self):
        sys.path.insert(0, str(SOURCE))
        from osp_mission.marker_approach import MarkerApproach
        approach = MarkerApproach()
        approach.observe_pose({7: (0., 0.)}, (1.4, 0.), 1_800_000_000., 1_800_000_000.1)
        self.assertTrue(approach.verified(7))

    def test_dry_run_keeps_ros_arguments_separate(self):
        result = subprocess.run(['bash', str(ROOT / 'run_robot_policy.sh'), '--domain', '7',
                                 '--namespace', 'robot1', '--rgb', 'camera/image', '--dry-run'],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('planner:=edge', result.stdout)
        self.assertIn('use_sim_time:=false', result.stdout)
        self.assertIn('/robot1/mission/start', result.stdout)

    def test_domain_is_required(self):
        import os
        environment = dict(os.environ)
        environment.pop('ROS_DOMAIN_ID', None)
        result = subprocess.run(['bash', str(ROOT / 'run_robot_policy.sh'), '--dry-run'],
                                env=environment, capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)


if __name__ == '__main__':
    unittest.main()
