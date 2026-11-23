"""Stage the joint wall/object selector over a verified Focus Continuity source."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile


HERE = Path(__file__).resolve().parent
PARENT = {
    'center_wall.py': '013cc8432c4a14aaf47b25d2672d94c5117d0351fd8c207d990e56ccf4028fac',
    'phased_hybrid.py': 'b962de92d15726667f876293f78bad889408c2388f9c6f46aeb4bb067bd06e90',
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def edit(path, substitutions):
    source = path.read_text()
    for before, after in substitutions:
        if source.count(before) != 1:
            raise ValueError(f'Missing unique integration point in {path.name}: {before}')
        source = source.replace(before, after, 1)
    compile(source, str(path), 'exec')
    path.write_text(source)


def install(parent_workspace, destination):
    destination = destination.resolve()
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temp:
        stage = Path(temp) / 'stage'
        shutil.copytree(parent_workspace / 'src/osp_mission', stage / 'src/osp_mission',
                        ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache', '._*'))
        shutil.copytree(parent_workspace / 'frozen-config', stage / 'frozen-config',
                        ignore=shutil.ignore_patterns('._*'))
        for manifest in ('focus_continuity_manifest.json', 'visual_survey_manifest.json'):
            shutil.copy2(parent_workspace / manifest, stage)
        package = stage / 'src/osp_mission/osp_mission'
        for name, checksum in PARENT.items():
            if digest(package / name) != checksum:
                raise ValueError(f'Unexpected Focus Continuity source: {name}')
        shutil.copy2(HERE / 'joint_surface.py', package)
        edit(package / 'center_wall.py', [
            ('from osp_mission.visual_survey import VisualSurvey',
             'from osp_mission.visual_survey import VisualSurvey\nfrom osp_mission.joint_surface import JointSurface'),
            ('        self.survey = VisualSurvey()', '        self.survey = VisualSurvey()\n        self.joint = JointSurface()'),
            ('        if alternative is not None:\n',
             """        reserved = alternative if alternative is not None and 'survey_wall' in alternative else None
        urgent_wall = reserved is not None and elapsed-self.survey.last_wall >= 150.
        if alternative is not None and (reserved is None or self.survey.loop_center is not None or urgent_wall):
"""),
            ('        if crowded and not boxes:\n',
             """        if not crowded:
            combined = self.joint.choose(grid, costmap, reachable, nav, distances, transform,
                                         self.survey, candidates['wall'], fresh or candidates['interior'],
                                         reserved, elapsed, duration)
            if combined is not None:
                self.phase = 'wall' if combined['r32_kind'] == 'wall_surface' else 'interior'
                self.reason = 'continue_local_surface_group' if combined['joint_continuation'] else 'joint_camera_gain_per_route'
                return combined
        if reserved is not None:
            self.phase, self.reason = 'wall', 'unobserved_wall_reserved'
            return reserved
        if crowded and not boxes:
"""),
            ('        self.batch.dispatched(goal)', '        self.batch.dispatched(goal)\n        self.joint.dispatched(goal, elapsed)'),
        ])
        edit(package / 'phased_hybrid.py', [
            ("        if cue is not None and (goal is None or not any(k in goal for k in ('distant_hint_id','survey_wall','survey_escape'))):",
             """        if cue is not None and (goal is None or 'joint_views' in goal
                                or not any(k in goal for k in ('distant_hint_id','survey_wall','survey_escape'))):"""),
            ("                   surface=goal.get('surface_map'), fitted_face=goal.get('fitted_face'),",
             """                   surface=goal.get('surface_map'), fitted_face=goal.get('fitted_face'),
                   joint_views=goal.get('joint_views'), joint_walk_m=goal.get('joint_walk_m'),
                   joint_route_m=goal.get('joint_route_m'), joint_score=goal.get('joint_score'),
                   joint_next_key=goal.get('joint_next_key'), joint_continuation=goal.get('joint_continuation'),"""),
        ])
        manifest = dict(version='1.5.2.14-joint-surface',
                        parent_sha256=digest(stage / 'focus_continuity_manifest.json'),
                        sources={name: digest(package / name) for name in (*PARENT, 'joint_surface.py')})
        (stage / 'joint_surface_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('parent_workspace', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.parent_workspace, args.destination)
