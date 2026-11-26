"""Install the opt-in strategy into a new workspace derived from general-surface Mixed."""
from pathlib import Path
import argparse
import shutil


def install(base,dest):
    source = base/'src/osp_mission'
    target = dest/'src/osp_mission'
    if dest.exists():
        raise FileExistsError(dest)
    mission = (source/'osp_mission/mission.py').read_text()
    edits = {
        'from osp_mission.hybrid import Hybrid':'from osp_mission.phased_hybrid import PhasedHybrid as Hybrid',
        'combined=hybrid.select(self,selected,data,viewed,(tr.transform.translation.x,tr.transform.translation.y,heading),memory_transform)':
        'combined=hybrid.select(self,selected,data,viewed,(tr.transform.translation.x,tr.transform.translation.y,heading),memory_transform,choices=choices)',
        'wall_due=(wall is not None and not protected':'wall_due=(wall is not None and self.hybrid is not None and self.hybrid.schedule.phase=="wall" and not protected',
        'self.near_pass.inspectable_route=inspectable_route(selected)':
        'self.near_pass.inspectable_route=False  # The phase scheduler owns surface observations.',
    }
    for old,new in edits.items():
        if mission.count(old)!=1:
            raise ValueError('Unexpected base mission: '+old)
        mission = mission.replace(old,new)
    dest.mkdir(parents=True)
    try:
        shutil.copytree(source,target,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
        package = target/'osp_mission'
        (package/'mission.py').write_text(mission)
        for name in ('phased_hybrid.py','center_wall.py'):
            shutil.copy2(Path(__file__).with_name(name),package/name)
    except Exception:
        shutil.rmtree(dest)
        raise


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('base',type=Path)
    p.add_argument('destination',type=Path)
    args=p.parse_args()
    install(args.base,args.destination)
