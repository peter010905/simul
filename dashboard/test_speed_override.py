"""Board top-speed choice: 'fast' adds --forward-045 to the next Mixed/MPPI preparation only."""
import tempfile
import unittest
from pathlib import Path

import robot_board_control as control


class SpeedOverrideTest(unittest.TestCase):
    def board(self):
        tmp = tempfile.mkdtemp()
        return control.BoardControl(Path(tmp), Path(tmp) / "ws", 4, state_dir=Path(tmp) / "state")

    def test_fast_applies_to_mixed_and_release_lineages_only(self):
        b = self.board()
        self.assertNotIn("--forward-045", b.command("prepare-thin-obstacles"))
        b.request("speed-fast")
        self.assertTrue(b.snapshot()["fast_speed"])
        self.assertEqual(b.snapshot()["speed_m_s"], .45)
        self.assertIn("--forward-045", b.command("prepare-thin-obstacles"))
        self.assertIn("--forward-045", b.command("prepare-v155-far-wall-slot"))
        self.assertNotIn("--forward-045", b.command("start-v3-mppi"))      # not a Mixed MPPI lineage
        self.assertNotIn("--forward-045", b.command("stop"))
        b.request("speed-normal")
        self.assertNotIn("--forward-045", b.command("prepare-thin-obstacles"))
        self.assertEqual(b.snapshot()["speed_m_s"], .30)


if __name__ == "__main__":
    unittest.main()
