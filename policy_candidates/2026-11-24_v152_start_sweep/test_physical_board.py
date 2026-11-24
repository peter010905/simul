from pathlib import Path
import sys
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "dashboard"))
from robot_board_control import BoardControl

@pytest.mark.parametrize("verb", ["prepare", "start"])
def test_board_routes_single_read_sweep_on_wired_hardware(tmp_path, verb):
    workspace = tmp_path / "v15231_start_sweep_640_1124_ws"
    setup = workspace / "install/setup.bash"
    setup.parent.mkdir(parents=True)
    setup.touch()
    board = BoardControl(tmp_path, tmp_path / "default", 4, state_dir=tmp_path)
    assert board.snapshot()["policies"] == [
        {"action": "start-sweep31", "label": "Mixed v1.5.2.31 · Start Sweep"}]
    with patch("robot_board_control.subprocess.Popen") as launch:
        board.spawn(verb + "-sweep31")
    args = launch.call_args.args[0]
    inner = args[args.index("--") + 1:]
    assert inner[3] == verb
    assert inner[inner.index("--workspace")+1] == str(workspace)
    assert "--share-camera" in inner and "--forward-045" not in inner
    env = launch.call_args.kwargs["env"]
    assert env["OSP_PUBLIC_MARKER_COUNT"] == "3"
    assert env["OSP_SURVEY_DISTANT_HINTS"] == "false"
    assert env["OSP_SHARE_CAMERA"] == "1"
    assert env["ROS_DOMAIN_ID"] == "4"
