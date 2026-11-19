"""Build a separate candidate on the frozen v2 package."""
import argparse
import hashlib
from pathlib import Path
import shutil


def edit(source, old, new):
    if source.count(old)!=1:
        raise ValueError(f'Unexpected source near {old}')
    return source.replace(old,new)


def stage(source, destination):
    views=(source/'box_views.py').read_bytes()
    if hashlib.sha256(views).hexdigest()!='1e040627cdb784faf8aa9079106bb40139ef349fb5398ceb279d7840429b07d6':
        raise ValueError('Unexpected box planner revision')
    tracker=(source/'two_views.py').read_bytes()
    if hashlib.sha256(tracker).hexdigest()!='d3cb327f1eb66e518fac1ebb70b1eb7e708a7c839c5292102772b6decc7d2311':
        raise ValueError('Unexpected tracker revision')
    tracking=edit(tracker.decode(),'        self.last_frame = stamp','        self.last_frame = stamp\n        self.corner_half_fov = max(0.,min(math.radians(25), fov/2-math.radians(4)))')
    modified=edit(views.decode(),'import math','import math\nfrom osp_mission.corner_pair import walking_distance, rate_corner')
    modified=edit(modified,'        nav = r32.connected_nav_cells','        steps = walking_distance(labels==labels[start],start,grid.resolution)\n        nav = r32.connected_nav_cells')
    modified=edit(modified,'            recent = [','            track = self.nearest(center_odom)\n            if track is None or not track.confirmed:\n                continue\n            recent = [')
    start=modified.index('                        behind = ')
    end=modified.index('                        goal = ',start)
    modified=modified[:start]+'''                        if cell not in steps:
                            continue
                        score, look, bearing, gain = rate_corner(grid,view,face,adjacent,side,transform[2],
                            track.seen,pose[2],steps[cell]*grid.resolution,self.corner_half_fov)
'''+modified[end:]
    modified=edit(modified,"box_point=face.tolist()", "box_point=look.tolist(),expected_faces=gain")
    out=destination/'osp_mission'
    out.mkdir(parents=True,exist_ok=False)
    for file in source.glob('*.py'):
        if file.name not in ('box_views.py','two_views.py'):
            (out/file.name).symlink_to(file.resolve())
    (out/'box_views.py').write_text(modified)
    (out/'two_views.py').write_text(tracking)
    shutil.copy(Path(__file__).with_name('corner_pair.py'),out/'corner_pair.py')


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('source',type=Path)
    p.add_argument('destination',type=Path)
    args=p.parse_args()
    stage(args.source,args.destination)
