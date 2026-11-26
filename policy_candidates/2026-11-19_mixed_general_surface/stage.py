"""Stage a separate two-step general-surface candidate from Mixed side-pass."""
import argparse
from pathlib import Path
import shutil


def once(text, old, new):
    if text.count(old) != 1:
        raise ValueError(f'Unexpected frozen source near {old!r}')
    return text.replace(old, new)


def stage(source, destination):
    package = destination / 'osp_mission'
    package.mkdir(parents=True, exist_ok=False)
    for path in source.glob('*.py'):
        if path.name not in ('mission.py', 'hybrid.py', 'near_pass_inspection.py',
                             'near_pass_runtime.py'):
            (package / path.name).symlink_to(path.resolve())

    mission = (source / 'mission.py').read_text()
    mission = once(mission, 'from osp_mission.side_route import side_face_route',
                   'from osp_mission.general_route import inspectable_route')
    mission = once(mission, 'getattr(self.near_pass,\'side_face_route\',False) and',
                   'getattr(self.near_pass,\'inspectable_route\',False) and')
    mission = once(mission, 'self.near_pass.side_face_route=side_face_route(selected)',
                   'self.near_pass.inspectable_route=inspectable_route(selected)')
    (package / 'mission.py').write_text(mission)

    near_pass = (source / 'near_pass_inspection.py').read_text()
    near_pass = once(near_pass, 'maximum_camera_range_m=1.2', 'maximum_camera_range_m=1.4')
    near_pass = once(near_pass, 'minimum_surface_length_m=.25', 'minimum_surface_length_m=.4')
    near_pass = once(near_pass, 'elapsed<15.', 'elapsed<10.')
    near_pass = once(near_pass,
                     '        eligible &= distances<readable_distance(focal)*incidence\n',
                     '        # A brief look can test a distant surface before marker readability.\n')
    near_pass = once(near_pass, '        eligible &= abs(bearing)>fov/2+.08\n',
                     '        # A surface already ahead may need a stationary camera dwell.\n')
    near_pass = once(near_pass, '            if abs(turn)<fov/2:continue',
                     '            if .001<abs(turn)<fov/2:continue')
    (package / 'near_pass_inspection.py').write_text(near_pass)

    runtime = (source / 'near_pass_runtime.py').read_text()
    runtime = once(runtime,
        '            goal=Spin.Goal();goal.target_yaw=turn;goal.time_allowance.sec=8',
        '''            if abs(turn)<.05:
                task.update(phase='dwell',frames_before=owner.camera_diagnostics['accepted'],
                    dwell_started=owner.now())
                self.event('front_dwell_started')
                return True
            goal=Spin.Goal();goal.target_yaw=turn;goal.time_allowance.sec=8''')
    (package / 'near_pass_runtime.py').write_text(runtime)

    hybrid = (source / 'hybrid.py').read_text()
    hybrid = once(hybrid, 'from osp_mission.hybrid_geometry import preview_geometry',
                  'from osp_mission.hybrid_geometry import preview_geometry\n'
                  'from osp_mission.general_surface import GeneralSurfaceFollowup')
    hybrid = once(hybrid, 'self.box_views = AngularBox()',
                  'self.box_views = AngularBox()\n        self.surface_followup = GeneralSurfaceFollowup()')
    hybrid = once(hybrid, '        if print_goal is not None or print_only:\n            return print_goal',
        '''        if print_goal is not None or print_only:
            return print_goal
        # A completed short look can justify at most one safe closer view of
        # the still-unobserved surface; a marker approach always won above.
        looks=getattr(owner.near_pass.policy,'history',())
        glance=looks[-1] if looks else None
        if glance and transform is not None and viewed is not None:
            try:
                glance_map=in_map(('odom',*glance['odom_camera'][:2]),transform)
                evidence=dict(glance,map_camera=glance_map)
                followup=self.surface_followup.choose(
                    grid,grid_from_message(msg),pose,viewed,evidence,len(looks)-1,
                    owner.now(),owner.p('duration')-(owner.now()-owner.started),veto)
                if followup is not None:return followup
            except Exception as exc:
                owner.get_logger().debug(f'General surface follow-up unavailable: {exc}')''')
    hybrid = once(hybrid, '        self.box_views.dispatched(goal, owner.now())',
                  '        self.box_views.dispatched(goal, owner.now())\n'
                  '        self.surface_followup.dispatched(goal, owner.now())')
    (package / 'hybrid.py').write_text(hybrid)
    for name in ('general_route.py', 'general_surface.py'):
        shutil.copy2(Path(__file__).with_name(name), package / name)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('source', type=Path)
    p.add_argument('destination', type=Path)
    a = p.parse_args()
    stage(a.source, a.destination)
