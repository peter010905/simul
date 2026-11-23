import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

from osp_mission import action_priority as action
from osp_mission import mission as runtime
from osp_mission._r32_navigation_core import Grid
from osp_mission.camera_work import ExposureQueue


class Handle:
    accepted = True

    def __init__(self):
        self.cancellations = 0

    def cancel_goal_async(self):
        self.cancellations += 1


class Future:
    def __init__(self, value):
        self.value = value

    def result(self):
        return self.value


class ActionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Compile the installed callbacks without constructing a ROS executor.
        path = Path(runtime.__file__)
        tree = ast.parse(path.read_text())
        mission = next(c for c in tree.body if isinstance(c, ast.ClassDef) and c.name == 'Mission')
        methods = ('cancel_action', 'accepted', 'completed')
        cls.callbacks = dict(priority=action.priority, SENSOR_REASONS=action.SENSOR_REASONS)
        selected = [m for m in mission.body if isinstance(m, ast.FunctionDef) and m.name in methods]
        exec(compile(ast.Module(body=selected, type_ignores=[]), str(path), 'exec'), cls.callbacks)

    def setUp(self):
        self.runner = SimpleNamespace(
            cancel_reason='', cancel_started=None, kind='nav', state='running',
            action_generation=2, goal_handle=Handle(), get_logger=lambda: SimpleNamespace(
                warning=lambda _: None, debug=lambda _: None), now=lambda: 3.,
            pending=False, contact_pending=True, recovery_pending=False,
            scan_after=False, excluded=[], destination=(1., 2.),
        )

    def test_bumper_beats_pending_optional_inspection_or_sensor_pause(self):
        cancel = self.callbacks['cancel_action']
        handle = self.runner.goal_handle
        cancel(self.runner, 'inspection_due')
        cancel(self.runner, 'processed_camera_stale')
        cancel(self.runner, 'bump_contact')
        cancel(self.runner, 'marker_hint')
        self.assertEqual(self.runner.cancel_reason, 'bump_contact')
        self.assertEqual(handle.cancellations, 1)
        self.callbacks['completed'](self.runner, Future(SimpleNamespace(status=5)), 2)
        self.assertTrue(self.runner.recovery_pending)
        self.assertFalse(self.runner.contact_pending)
        self.assertFalse(self.runner.scan_after)
        self.assertIsNone(self.runner.resume_action)

    def test_obsolete_action_cannot_clear_current_action(self):
        old = Handle()
        current = self.runner.goal_handle
        self.runner.pending = True
        self.callbacks['accepted'](self.runner, Future(old), 1)
        self.assertEqual(old.cancellations, 1)
        self.assertTrue(self.runner.pending)
        self.callbacks['completed'](self.runner, Future(SimpleNamespace(status=5)), 1)
        self.assertIs(self.runner.goal_handle, current)

    def test_exposure_queue_drops_aged_frame_and_uses_newest_ready(self):
        q = ExposureQueue()
        q.offer('old', None, 1.0, 1.)
        q.offer('incomplete TF', None, 1.4, 1.4)
        frame = q.take(1.45, lambda _: False)
        self.assertIsNone(frame)
        q.offer('ready', None, 1.6, 1.6)
        self.assertEqual(q.take(1.65, lambda row: row.message == 'ready').message, 'ready')
        self.assertEqual(q.counts['expired'], 1)
        self.assertFalse(q.offer('late duplicate', None, 1.4, 1.7))


class VisibilityTests(unittest.TestCase):
    def test_vectorized_rays_keep_the_existing_visibility_rule(self):
        rng = np.random.default_rng(417608489)
        data = np.zeros((60, 60), np.int8)
        cells = rng.integers(0, 60, size=(130, 2))
        data[cells[:, 0], cells[:, 1]] = rng.choice((-1, 26, 99, 100), len(cells))
        grid = Grid(data, .05, (-.25, .30, .31))

        def prior(a, b, allow_unknown):
            n = max(2, int(np.linalg.norm(b - a) / grid.resolution) * 2)
            for p in np.linspace(a, b, n)[:-2]:
                row, col = grid.index(*p)
                if not grid.inside(row, col):
                    return False
                cell = grid.cells[row, col]
                if cell > 25 or (cell < 0 and not allow_unknown):
                    return False
            return True

        points = rng.uniform(-.7, 2.8, size=(12000, 2, 2))
        for i, (a, b) in enumerate(points):
            allow = bool(i % 2)
            self.assertEqual(grid.line_clear(a, b, allow), prior(a, b, allow))


if __name__ == '__main__':
    unittest.main()
