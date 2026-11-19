"""Create an independent simulation overlay from the existing box-view revision."""
import argparse
from pathlib import Path
import shutil


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError(f'Unexpected base revision at {old!r}')
    return text.replace(old, new)


def stage(source, destination):
    hybrid = (source/'hybrid.py').read_text()
    hybrid = replace_once(hybrid, 'from osp_mission.box_views import BoxViews', 'from osp_mission.two_views import TwoViews')
    hybrid = replace_once(hybrid, 'self.box_views = BoxViews()', 'self.box_views = TwoViews()')
    hybrid = replace_once(hybrid, 'self.drive.inspect_boxes=(transform is not None and owner.now()-self.last_box>=25.)', 'self.drive.inspect_boxes=False')
    mission = (source/'mission.py').read_text()
    marker = "            self.camera_diagnostics['coverage_updates']+=1\n            self.pending_camera=None"
    mission = replace_once(mission, marker, marker+'''
            if getattr(self,'hybrid',None) is not None:
                views=self.hybrid.box_views
                exposure=stamp.nanoseconds/1e9
                if exposure-views.last_frame>=.4:
                    from osp_mission.r32_drive import grid_from_message
                    from osp_mission.wall_route import yaw as map_yaw
                    mt=self.tf.lookup_transform('odom',self.p('map_frame'),stamp).transform
                    views.observe(grid_from_message(grid),(camera.transform.translation.x,camera.transform.translation.y,yaw),
                                  endpoints,(mt.translation.x,mt.translation.y,map_yaw(mt.rotation)),
                                  exposure,2*math.atan(m.width/(2*m.k[0])))
                    self.camera_diagnostics['two_view_frames']=self.camera_diagnostics.get('two_view_frames',0)+1
                    self.camera_diagnostics['confirmed_boxes']=sum(t.confirmed for t in views.tracks)
                    self.camera_diagnostics['observed_box_faces']=sum(len(t.seen) for t in views.tracks if t.confirmed)
''')
    package = destination/'osp_mission'
    package.mkdir(parents=True, exist_ok=False)
    for path in source.glob('*.py'):
        if path.name not in ('hybrid.py','mission.py'):
            (package/path.name).symlink_to(path.resolve())
    (package/'hybrid.py').write_text(hybrid)
    (package/'mission.py').write_text(mission)
    shutil.copy(Path(__file__).with_name('two_views.py'),package/'two_views.py')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('source',type=Path)
    parser.add_argument('destination',type=Path)
    args = parser.parse_args()
    stage(args.source,args.destination)
