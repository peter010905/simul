""".66: interior samples take the smoothed surface normal; lap stops 1.2-1.8 m; far-wall Nav2."""
import os
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get('OSP_OVERLAY', HERE / 'overlay'))
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(HERE))


def test_interior_samples_use_the_surface_normal():
    source = (ROOT / 'osp_mission/center_wall.py').read_text()
    assert "if wall_normals is not None:\n                normal = self.survey.wall_normal(" in source
    assert "if kind == 'wall' and wall_normals is not None" not in source


def test_lap_standoffs():
    from osp_mission import wall_lap
    assert wall_lap.LAP_STANDOFFS_M == (1.2, 1.5, 1.8) and wall_lap.LAP_PREFERRED_M == 1.5
    assert wall_lap.EXTENDED_LAP_S == 210.                      # .65 kept
    assert 'for stand in (.9,1.2,1.5)' not in (ROOT / 'osp_mission/wall_lap.py').read_text()


def test_far_wall_nav2():
    import install
    plugin = '        plugin: nav2_costmap_2d::InflationLayer\n'
    text = ('global_costmap:\n  global_costmap:\n    ros__parameters:\n' + install.GLOBAL_OLD + plugin +
            'local_costmap:\n  local_costmap:\n    ros__parameters:\n' + install.GLOBAL_OLD + plugin)
    out = install.far_wall_nav2(text)
    head, local = out.split('\nlocal_costmap:', 1)
    assert install.GLOBAL_NEW in head and install.GLOBAL_OLD in local


if __name__ == '__main__':
    test_interior_samples_use_the_surface_normal()
    test_lap_standoffs()
    test_far_wall_nav2()
    print('ok')
