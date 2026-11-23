"""Keep planned exploration transit until arrival unless stronger evidence interrupts."""
import argparse
import importlib.util
import json
import tempfile
from pathlib import Path


HERE = Path(__file__).resolve().parent
parent = HERE.parent / '2026-11-24_v152_open_regions/install.py'
spec = importlib.util.spec_from_file_location('open_regions_installer', parent)
open_regions = importlib.util.module_from_spec(spec)
spec.loader.exec_module(open_regions)


def install(parent_workspace, destination):
    destination = destination.resolve()
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as tmp:
        stage = Path(tmp) / 'stage'
        open_regions.install(parent_workspace, stage)
        mission = stage / 'src/osp_mission/osp_mission/mission.py'
        open_regions.stable.edit(mission, [
            ("        wall=getattr(self,'wall_look',None)\n        wall_due=",
             """        active_motion = bool(self.goal_handle or self.pending)
        can_pause_for_survey = not active_motion or flow is None or flow.allow_mid_route_wall_inspection
        periodic_due = bool(guard is not None and can_pause_for_survey and guard.due(now))
        wall=getattr(self,'wall_look',None)
        wall_due="""),
            ('wall_due=(wall is not None and self.hybrid is not None',
             'wall_due=(wall is not None and can_pause_for_survey and self.hybrid is not None'),
            ('                    and (guard.due(now) or bearing_fresh or wall_due)):',
             '                    and (periodic_due or bearing_fresh or wall_due)):'),
            ('now>=self.spin_blocked_until and (guard.due(now) or bearing_fresh or wall_due):',
             'now>=self.spin_blocked_until and (periodic_due or bearing_fresh or wall_due):'),
        ])
        digest = open_regions.stable.regional.joint.digest
        manifest = dict(version='1.5.2.18-finish-transit',
                        parent_sha256=digest(stage / 'open_regions_manifest.json'),
                        mission_sha256=digest(mission))
        (stage / 'finish_transit_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('parent_workspace', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.parent_workspace, args.destination)
