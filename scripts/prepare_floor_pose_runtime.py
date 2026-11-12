"""Correct the visual floor's relative pose in a NEW frozen runtime."""
import argparse
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]


def world_code(code):
    if 'floor_visual=scene.find' in code:
        raise ValueError('Floor relative pose is already corrected; do not apply twice')
    old = "        ET.SubElement(ET.SubElement(geometry,'mesh'),'uri').text=str(mesh)"
    if code.count(old) != 1:
        raise ValueError('Review changed floor mesh patch anchor')
    return code.replace(old, old+'''
        # Mesh vertices are world coordinates. Cancel the enclosing floor
        # model's centre translation and -0.06 m vertical offset.
        floor_visual=scene.find("model[@name='floor']/link/visual")
        ET.SubElement(floor_visual,'pose').text=f'{-w/2} {-h/2} 0.06 0 0 0'
''')


def prepare(source, destination):
    if destination.exists():
        raise FileExistsError('Existing runtimes are immutable; use a new destination')
    package = destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission', package,
                    ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    path = package/'osp_mission/world.py'
    path.write_text(world_code(path.read_text()))
    shutil.copy2(ROOT/'scripts/test_floor_pose_runtime.py',package/'test/test_floor_pose_runtime.py')
    return destination


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--destination',type=Path,required=True)
    args = parser.parse_args()
    print(prepare(args.source.resolve(),args.destination.resolve()))
