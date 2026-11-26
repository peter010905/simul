"""Fork the glass exclusion workspace with opt-in white evidence for thin obstacles."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile

PACKAGE = Path('src/osp_mission/osp_mission')
PATCHES = {
    'detector.py': (
        ('            small=[]\n',
         '            from osp_mission.white_legs import publish_white_patches\n'
         '            try:\n'
         '                publish_white_patches(self, msg, image, k)\n'
         '            except Exception as exc:\n'
         '                self.get_logger().debug(f\'White patch projection unavailable: {exc}\')\n'
         '            small=[]\n'),
    ),
    'hybrid.py': (
        ("        nearest = min(self.tracks,key=lambda r:np.linalg.norm(point-r['point']),default=None)\n",
         "        white_leg = evidence.get('white_leg') is True\n"
         "        matching = (r for r in self.tracks if r.get('white_leg', False) == white_leg)\n"
         "        nearest = min(matching,key=lambda r:np.linalg.norm(point-r['point']),default=None)\n"),
        ('self.tracks.append(dict(point=point,observer=observer,stamp=stamp,count=1))',
         'self.tracks.append(dict(point=point,observer=observer,stamp=stamp,count=1,white_leg=white_leg))'),
        ('    def choose(self, grid, cost, pose, transform, targets, now, veto):',
         '    def choose(self, grid, cost, pose, transform, targets, now, veto, *, white_filter=None):'),
        ("            point=np.asarray(in_map(('odom',*r['point']),transform))\n",
         "            point=np.asarray(in_map(('odom',*r['point']),transform))\n"
         "            if r.get('white_leg') and (white_filter is None or not white_filter(point)):\n"
         '                continue\n'),
        ("                          print_odom=r['point'].tolist())",
         "                          print_odom=r['point'].tolist(), white_leg=r.get('white_leg', False))"),
    ),
    'phased_hybrid.py': (
        ('        cue = self.memory.choose(grid,cost,pose,transform,owner.targets,clock,veto)',
         '        def white_filter(point):\n'
         '            self.schedule.thin_obstacles.update(grid)\n'
         '            return (self.schedule.thin_obstacles.near(point)\n'
         '                    and not self.schedule.glass_wall.includes(point))\n\n'
         '        cue = self.memory.choose(grid,cost,pose,transform,owner.targets,clock,veto,white_filter=white_filter)'),
        ("            self.schedule.reason = 'local_print_candidate'",
         "            self.schedule.reason = 'white_leg_candidate' if cue.get('white_leg') else 'local_print_candidate'"),
        ("                   available=self.schedule.counts,kind=goal.get('r32_kind','flow_or_marker'),",
         "                   available=self.schedule.counts,kind=goal.get('r32_kind','flow_or_marker'),\n"
         "                   white_leg=bool(goal.get('white_leg')), white_point=goal.get('print_point') if goal.get('white_leg') else None,"),
    ),
    'center_wall.py': (
        ('from osp_mission.marker_cooldown import exploration_gain\n',
         'from osp_mission.marker_cooldown import exploration_gain\nfrom osp_mission.white_legs import ThinObstacles\n'),
        ('        self.glass_wall = GlassWallMask()\n',
         '        self.glass_wall = GlassWallMask()\n        self.thin_obstacles = ThinObstacles()\n'),
        ('        samples = self.glass_wall.filter_samples(samples, grid)\n',
         '        samples = self.glass_wall.filter_samples(samples, grid)\n'
         '        self.thin_obstacles.update(grid)\n'
         '        thin_skipped = 0\n'),
        ('        for center, faces, seen_angles in fitted:\n',
         '        for center, faces, seen_angles in fitted:\n'
         '            if self.thin_obstacles.near(center):\n'
         '                thin_skipped += len(faces)\n'
         '                continue\n'),
        ('            row,col = remaining[len(remaining)//2]\n',
         '            row,col = remaining[len(remaining)//2]\n'
         "            if kind == 'interior' and self.thin_obstacles.near(grid.world(row, col), 0.):\n"
         '                thin_skipped += 1\n'
         '                continue\n'),
        ("        self.counts['marker_cooled'] = marker_cooled\n",
         "        self.counts['marker_cooled'] = marker_cooled\n"
         "        self.counts['thin_surface_skipped'] = thin_skipped\n"),
    ),
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(parent, destination):
    parent, destination = parent.resolve(), destination.resolve()
    manifest = parent / 'glass_wall_manifest.json'
    source = json.loads(manifest.read_text())
    if source['version'] != '1.5.2.32-glass-wall':
        raise ValueError('Expected Glass Wall parent')
    for name, expected in source['sources'].items():
        if digest(parent / PACKAGE / name) != expected:
            raise ValueError(f'Parent file changed: {name}')
    if destination.exists():
        raise FileExistsError(destination)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temporary:
        stage = Path(temporary) / 'stage'
        stage.mkdir()
        for name in ('src', 'frozen-config'):
            shutil.copytree(parent / name, stage / name, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
        for path in parent.glob('*_manifest.json'):
            shutil.copyfile(path, stage / path.name)
        for name, changes in PATCHES.items():
            path = stage / PACKAGE / name
            code = path.read_text()
            for old, new in changes:
                if code.count(old) != 1:
                    raise ValueError(f'Unexpected {name} source at {old.strip()}')
                code = code.replace(old, new)
            compile(code, str(path), 'exec')
            path.write_text(code)
        shutil.copyfile(Path(__file__).with_name('white_legs.py'), stage / PACKAGE / 'white_legs.py')
        (stage / 'white_legs_manifest.json').write_text(json.dumps({
            'version': '1.5.2.33-white-legs',
            'parent_sha256': digest(manifest),
            'sources': {name: digest(stage / PACKAGE / name) for name in [*PATCHES, 'white_legs.py']},
        }, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('parent', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.parent, args.destination)
