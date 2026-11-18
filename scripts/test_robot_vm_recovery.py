"""Verify VM recovery decisions without controlling a VM or robot."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('recovery', Path(__file__).with_name('robot_vm_recovery.py'))
recovery = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recovery)


def result(code=0):
    return SimpleNamespace(returncode=code, stderr='', stdout='')


class RecoveryTests(unittest.TestCase):
    def test_healthy_vm_never_restarts(self):
        with patch.object(recovery, 'vm_command', return_value=result()), patch.object(recovery, 'run') as run:
            recovery.recover('vm', '/repo', 'robot')
        run.assert_not_called()

    def test_disconnected_robot_never_restarts(self):
        with patch.object(recovery, 'vm_command', return_value=result(1)), patch.object(recovery, 'run', return_value=result(1)) as run:
            with self.assertRaisesRegex(RuntimeError, 'Mac cannot reach'):
                recovery.recover('vm', '/repo', 'robot')
        self.assertEqual(run.call_count, 1)

    def test_active_session_blocks_power_cycle(self):
        with patch.object(recovery, 'vm_command', side_effect=[result(1), result(1)]), patch.object(recovery, 'run', return_value=result()) as run:
            with self.assertRaisesRegex(RuntimeError, 'session may be active'):
                recovery.recover('vm', '/repo', 'robot')
        self.assertEqual(run.call_count, 1)

    def test_isolated_vm_reconnects_once_and_restores_board(self):
        with patch.object(recovery, 'vm_command', side_effect=[result(1), result(), result(), result()]) as vm, patch.object(recovery, 'run', return_value=result()) as run:
            recovery.recover('vm', '/repo', 'robot')
        self.assertEqual([call.args[0] for call in run.call_args_list], [
            ['ping','-c','1','-W','1000','robot'], ['multipass','stop','vm'], ['multipass','start','vm']])
        self.assertIn('robot_board.py', vm.call_args.args[1])
        self.assertNotIn('/mission/start', vm.call_args.args[1])


if __name__ == '__main__':
    unittest.main()
