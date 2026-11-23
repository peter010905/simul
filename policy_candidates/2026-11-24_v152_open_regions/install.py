"""Add a new-region option between bounded local visits."""
import argparse
import importlib.util
import json
import shutil
import tempfile
from pathlib import Path


HERE = Path(__file__).resolve().parent
base = HERE.parent / '2026-11-24_v152_stable_survey/install.py'
spec = importlib.util.spec_from_file_location('stable_survey_installer', base)
stable = importlib.util.module_from_spec(spec)
spec.loader.exec_module(stable)


def install(parent_workspace, destination):
    destination = destination.resolve()
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temp:
        stage = Path(temp) / 'stage'
        stable.install(parent_workspace, stage)
        package = stage / 'src/osp_mission/osp_mission'
        shutil.copy2(HERE / 'open_regions.py', package)
        stable.edit(package / 'center_wall.py', [
            ('from osp_mission.regional_surface import RegionalSurface',
             'from osp_mission.regional_surface import RegionalSurface\n'
             'from osp_mission.open_regions import choose_unvisited'),
            ('        self.joint = RegionalSurface()', '        self.joint = RegionalSurface()\n        self.last_new_region = 0.'),
            ('            if combined is not None:\n                self.phase',
             """            if (not (combined and combined.get('joint_continuation'))
                    and elapsed-self.last_new_region >= 90. and duration-elapsed >= 55.):
                region = choose_unvisited(choices, grid, costmap, nav, reachable, distances, transform,
                                          self.survey.trail, elapsed, duration-elapsed, veto)
                if region is not None:
                    self.phase, self.reason = 'interior', 'open_new_region_for_camera_coverage'
                    return region
            if combined is not None:
                self.phase"""),
            ('        self.joint.dispatched(goal, elapsed)', """        self.joint.dispatched(goal, elapsed)
        if goal.get('survey_region'):
            self.last_new_region = elapsed"""),
        ])
        stable.edit(package / 'phased_hybrid.py', [
            ("                   region_anchor=goal.get('region_anchor'), region_face=goal.get('region_face'),",
             """                   region_anchor=goal.get('region_anchor'), region_face=goal.get('region_face'),
                   new_region_distance_m=goal.get('new_region_distance_m'), new_region_path_m=goal.get('new_region_path_m'),"""),
        ])
        digest = stable.regional.joint.digest
        manifest = dict(version='1.5.2.17-open-regions', parent_sha256=digest(stage/'stable_survey_manifest.json'),
                        sources={name: digest(package/name)
                                 for name in ('center_wall.py', 'phased_hybrid.py', 'open_regions.py')})
        (stage/'open_regions_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('parent_workspace', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.parent_workspace, args.destination)
