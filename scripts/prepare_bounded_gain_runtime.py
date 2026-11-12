"""Remove full-circle observation reward from the bounded-sweep edge policy."""
import argparse
from pathlib import Path
import shutil

ROOT=Path(__file__).resolve().parents[1]
ANCHOR='''            if len(indices)>counts[best]*1.5:
                count=len(indices)*resolution;angle=2*math.pi;yaw=math.atan2(py-robot[1],px-robot[0])
                counted=indices
'''


def prepare(source,destination):
    if destination.exists():raise FileExistsError('Create a new immutable candidate')
    original=(source/'src/osp_mission/osp_mission/edge_policy.py').read_text()
    if original.count(ANCHOR)!=1:raise ValueError('Review changed observation gain implementation')
    mission=(source/'src/osp_mission/osp_mission/mission.py').read_text()
    if 'min(abs(requested),2*math.pi/3)' not in mission or "full_scans=True" in mission:
        raise ValueError('Verify bounded native edge sweep routing')
    package=destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission',package,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    (package/'osp_mission/edge_policy.py').write_text(original.replace(ANCHOR,
        '            # Reward only this bounded sector; motion never scans a full circle here.\n'))
    shutil.copy2(ROOT/'scripts/test_bounded_gain_runtime.py',package/'test/test_bounded_gain_runtime.py')
    return destination


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--destination',type=Path,required=True)
    args=parser.parse_args();print(prepare(args.source.resolve(),args.destination.resolve()))
