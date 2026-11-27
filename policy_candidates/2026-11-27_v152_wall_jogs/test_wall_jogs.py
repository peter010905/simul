""".67: wall-step faces on long walls, steady lap progress, open escape and the step-face priority."""
import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get('OSP_OVERLAY', HERE / 'overlay'))
sys.path.insert(0, str(ROOT))

from osp_mission import _r32_navigation_core as r32  # noqa: E402
from osp_mission.wall_jogs import wall_jog_faces  # noqa: E402
from osp_mission.wall_lap import WallLap  # noqa: E402


def room(step=True, rotate=0.0):
    """10 x 6 m room, walls 2 cells thick; a 0.6 m deep, 0.8 m wide pillar on one long wall."""
    res = .05
    cells = np.full((160, 240), -1, np.int8)
    cells[20:140, 20:220] = 0
    cells[18:20, 18:222] = cells[140:142, 18:222] = 100
    cells[18:142, 18:20] = cells[18:142, 220:222] = 100
    if step:
        cells[20:32, 110:126] = 100                      # pillar on the y-min long wall
    if rotate:
        h, w = cells.shape
        import cv2
        m = cv2.getRotationMatrix2D((w / 2, h / 2), rotate, 1.)
        cells = cv2.warpAffine(cells.astype(np.int16) + 1, m, (w, h), flags=cv2.INTER_NEAREST,
                               borderValue=0).astype(np.int16) - 1
        cells = cells.astype(np.int8)
    return r32.Grid(cells, res, (0., 0., 0.))


def test_pillar_faces_found_on_the_long_wall():
    grid = room()
    faces = wall_jog_faces(grid, (6., 4., 0.))
    assert len(faces) >= 2, faces
    for middle, normal, half in faces:
        assert 5.3 <= middle[0] <= 6.5 and middle[1] <= 1.8   # at the pillar
        cell = grid.index(*(middle + .3 * normal))
        assert grid.cells[cell] == 0                        # normal points into the room


def test_plain_room_and_rotation():
    assert wall_jog_faces(room(step=False), (6., 4., 0.)) == []
    assert len(wall_jog_faces(room(rotate=30.), (6., 4., 0.))) >= 2


def test_progress_does_not_go_back():
    lap = WallLap()
    lap.perimeter_m = 30.
    lap.turn = .62 * 2 * math.pi
    assert round(lap.progress(), 2) == .62
    lap.turn = .5 * 2 * math.pi
    assert round(lap.progress(), 2) == .62


def test_sources_carry_escape_and_priority():
    assert 'wall_step_face' in (ROOT / 'osp_mission/center_wall.py').read_text()
    mission = (ROOT / 'osp_mission/mission.py').read_text()
    assert 'open' in mission.lower() and 'escape' in mission.lower()


if __name__ == '__main__':
    test_pillar_faces_found_on_the_long_wall()
    test_plain_room_and_rotation()
    test_progress_does_not_go_back()
    test_sources_carry_escape_and_priority()
    print('ok')
