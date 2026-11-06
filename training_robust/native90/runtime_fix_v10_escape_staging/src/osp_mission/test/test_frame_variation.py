"""Coordinate changes must preserve obstacles and camera visibility."""

import math
import numpy as np

from osp_mission.fast_sim import SearchEnv
from osp_mission.frame_variation import rotate_scene_frame
from osp_mission.policy import wrap
from osp_mission.scenarios import rasterize


def scene():
    return dict(seed=1,start=[2.45,2.45,0.],obstacles=[],
                targets=[dict(id=7,x=4.45,y=2.45,yaw=math.pi,size=.3)])


def test_four_map_rotations_restore_collision_grid_and_relative_geometry():
    original=scene();grid=rasterize(original,.2);rotated=original;copy=grid.copy()
    for _ in range(4):rotated,copy=rotate_scene_frame(rotated,copy,.2,1)
    np.testing.assert_array_equal(copy,grid)
    np.testing.assert_allclose(rotated['start'],original['start'],atol=1e-12)
    target=rotated['targets'][0];reference=original['targets'][0]
    np.testing.assert_allclose([target['x'],target['y']],[reference['x'],reference['y']],atol=1e-12)
    assert abs(wrap(target['yaw']-reference['yaw']))<1e-12
    assert original==scene()


def test_marker_readability_does_not_change_with_map_frame_rotation():
    for quarter_turns in range(4):
        env=SearchEnv(scene(),seed=5,camera_width=960,camera_hz=5,
                      detection_dropout=0.,frame_quarter_turns=quarter_turns)
        for stamp in [.2,.4]:env.elapsed=stamp;env.scan()
        assert env.found=={7}
