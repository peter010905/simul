"""Stage bounded en-route side-face inspection from the frozen corner candidate."""
import argparse
from pathlib import Path
import shutil


def once(source, old, new):
    if source.count(old) != 1:
        raise ValueError(f'Unexpected mission source near {old!r}')
    return source.replace(old, new)


def stage(source, destination):
    package = destination / 'osp_mission'
    package.mkdir(parents=True, exist_ok=False)
    for path in source.glob('*.py'):
        if path.name != 'mission.py':
            (package / path.name).symlink_to(path.resolve())
    mission = (source / 'mission.py').read_text()
    mission = once(mission,
        'from osp_mission.near_pass_runtime import NearPassRuntime',
        'from osp_mission.near_pass_runtime import NearPassRuntime\nfrom osp_mission.side_route import side_face_route')
    mission = once(mission,
        "        if getattr(self,'near_pass',None) is not None:self.near_pass.protected_navigation=any(k in selected for k in ('approach_id','focus_id','print_point','box_point'))",
        "        if getattr(self,'near_pass',None) is not None:\n"
        "            # Preserve confirmed markers and print revisits. A box/face\n"
        "            # transit may pause for another visible side and resume.\n"
        "            self.near_pass.protected_navigation=any(k in selected for k in ('approach_id','focus_id','print_point'))\n"
        "            self.near_pass.side_face_route=side_face_route(selected)")
    mission = once(mission,
        "                  (flow is None or flow.allow_mid_route_wall_inspection)):self.near_pass.request()",
        "                  getattr(self.near_pass,'side_face_route',False) and\n"
        "                  (flow is None or flow.allow_mid_route_wall_inspection)):self.near_pass.request()")
    (package / 'mission.py').write_text(mission)
    shutil.copy2(Path(__file__).with_name('side_route.py'), package / 'side_route.py')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('source', type=Path)
    p.add_argument('destination', type=Path)
    a = p.parse_args()
    stage(a.source, a.destination)
