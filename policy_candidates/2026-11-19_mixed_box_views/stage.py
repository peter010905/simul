"""Stage the box-view extension separately from the pinned Mixed workspace."""
from pathlib import Path
import argparse
import hashlib
import shutil


def stage(source, destination):
    package=destination/'osp_mission'
    package.mkdir(parents=True,exist_ok=False)
    original=(source/'hybrid.py').read_bytes()
    if hashlib.sha256(original).hexdigest()!='e6ac6bb9f7640e24e8d4fffa74ff6c691516d00de3a294636e840eaf090347aa':
        raise ValueError('Expected the released Mixed hybrid')
    for path in source.glob('*.py'):
        if path.name!='hybrid.py':
            (package/path.name).symlink_to(path.resolve())
    shutil.copy(Path(__file__).with_name('box_views.py'),package/'box_views.py')
    text=original.decode().replace('        self.drive=R32Drive()', '''        from osp_mission.box_views import BoxViews
        self.box_views = BoxViews()
        self.next_box_probe = 0.
        self.drive=R32Drive()''')
    a=text.index("        if not any(r['count']>=3")
    b=text.index('        try:',a)
    text=text[:a]+'''        fresh_print = any(r['count']>=3 and 0<=stamp-r['stamp']<=1. for r in self.memory.tracks)
        probe_box = now>=self.next_box_probe and now-self.box_views.last_choice>=10.
        if not fresh_print and not probe_box:
            return False
        if probe_box:
            self.next_box_probe=now+3.
'''+text[b:]
    text=text.replace('print_only=True)', 'print_only=True, box_probe=probe_box)')
    text=text.replace('*, print_only=False):','*, print_only=False, box_probe=False):')
    text=text.replace('        if print_goal is not None or print_only:', '''        if print_goal is None and (not print_only or box_probe):
            box = self.box_views.choose(grid,grid_from_message(msg),pose,transform,owner.now(),owner.targets,veto)
            if box is not None:
                return box
        if print_goal is not None or print_only:''')
    text=text.replace('        self.drive.dispatched(goal)', '''        self.box_views.dispatched(goal, owner.now())
        self.drive.dispatched(goal)''')
    (package/'hybrid.py').write_text(text)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('source',type=Path)
    p.add_argument('destination',type=Path)
    args=p.parse_args()
    stage(args.source,args.destination)
