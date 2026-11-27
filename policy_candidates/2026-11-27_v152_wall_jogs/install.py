"""Create an isolated .67 wall-jogs workspace (.66 + wall-step faces, open escape, steady lap progress); never update a parent or board default.

Policy: lap stops 1.2-1.8 m from the wall (wall_lap.py); interior samples use the smoothed
surface normal (center_wall.py).
Nav2: the global costmap's inflation is widened so planned paths keep away from walls where the
room allows (radius 0.45 -> 0.90 m, cost scaling 4.0 -> 2.0). Cells stay passable (< inscribed
cost), so narrow aisles remain reachable. The local costmap and controller are unchanged.
The same Nav2 file is written to frozen-config (used by the Gazebo twin) and to
nav2_override.yaml (used by robot_session.py only when that file exists in the workspace).
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile

HERE = Path(__file__).resolve().parent
PACKAGE = Path('src/osp_mission/osp_mission')
NAV2 = Path('frozen-config/nav2_hardware_profile.yaml')
# The shared hardware profile robot_profiles/wired_flow_v3/nav2_hardware_profile.yaml; the Gazebo
# stages carry the same file in frozen-config. Older hardware workspaces hold a stale copy there.
NAV2_BASE_SHA256 = 'cd1a23bb40bcdfb4481f74d37e6ef91a4ffb93aab7214fd818c3bf4845d94cd6'
GLOBAL_OLD = ("      inflation_layer:\n        cost_scaling_factor: 4.0\n        inflation_radius: 0.45\n")
GLOBAL_NEW = ("      inflation_layer:\n        cost_scaling_factor: 2.0\n        inflation_radius: 0.9\n")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def far_wall_nav2(text):
    head, marker, rest = text.partition("global_costmap:\n  global_costmap:\n")
    if not marker or rest.count(GLOBAL_OLD) < 1:
        raise ValueError('Unexpected Nav2 profile: global inflation block not found')
    end = rest.find("\nlocal_costmap:")
    block, tail = rest[:end], rest[end:]
    if block.count(GLOBAL_OLD) != 1:
        raise ValueError('Unexpected Nav2 profile: global inflation block not unique')
    return head + marker + block.replace(GLOBAL_OLD, GLOBAL_NEW) + tail


def install(parent, destination, nav2_base, check_manifest=True):
    parent, destination = parent.resolve(), destination.resolve()
    if sha(nav2_base) != NAV2_BASE_SHA256:
        raise ValueError('Nav2 base is not the shared hardware profile: ' + str(nav2_base))
    manifest = parent / 'face_normals_manifest.json'
    if check_manifest and json.loads(manifest.read_text())['version'] != '1.5.2.66-face-normals':
        raise ValueError('Expected the .66 face-normals parent')
    expected = json.loads((HERE / 'parent_sources.json').read_text())
    for name, value in expected.items():
        if sha(parent / name) != value:
            raise ValueError('Parent source changed: ' + name)
    if destination.exists():
        raise FileExistsError(destination)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temporary:
        stage = Path(temporary) / 'stage'
        stage.mkdir()
        for name in ('src', 'frozen-config'):
            shutil.copytree(parent/name, stage/name,
                            ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
        for file in parent.glob('*.json'):
            shutil.copyfile(file, stage/file.name)
        for file in (HERE/'overlay/osp_mission').glob('*.py'):
            compile(file.read_text(), str(file), 'exec')
            shutil.copyfile(file, stage/PACKAGE/file.name)
        nav2 = far_wall_nav2(Path(nav2_base).read_text())
        (stage/NAV2).write_text(nav2)
        (stage/'nav2_override.yaml').write_text(nav2)
        sources = {f.name: sha(stage/PACKAGE/f.name) for f in (HERE/'overlay/osp_mission').glob('*.py')}
        sources['nav2_override.yaml'] = sha(stage/'nav2_override.yaml')
        (stage/'wall_jogs_manifest.json').write_text(json.dumps(dict(
            version='1.5.2.67-wall-jogs',
            parent_sha256=sha(manifest) if manifest.exists() else None,
            sources=sources, physical_drive=False, board_default_changed=False), indent=2)+'\n')
        stage.rename(destination)


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('parent',type=Path)
    p.add_argument('destination',type=Path)
    p.add_argument('nav2_base',type=Path,help='robot_profiles/wired_flow_v3/nav2_hardware_profile.yaml')
    p.add_argument('--copy-staged-parent', action='store_true',
                   help='parent is a copy-staged Gazebo stage without the .66 manifest')
    a=p.parse_args()
    install(a.parent,a.destination,a.nav2_base,check_manifest=not a.copy_staged_parent)
