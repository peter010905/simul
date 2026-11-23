"""Build a separate, bounded regional survey variant from a Focus workspace."""
import argparse
import importlib.util
import json
import shutil
import tempfile
from pathlib import Path


HERE = Path(__file__).resolve().parent
JOINT_INSTALL = HERE.parent / '2026-11-24_v152_joint_surface/install.py'
spec = importlib.util.spec_from_file_location('joint_surface_installer', JOINT_INSTALL)
joint = importlib.util.module_from_spec(spec)
spec.loader.exec_module(joint)


def install(parent_workspace, destination):
    destination = destination.resolve()
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as tmp:
        stage = Path(tmp) / 'stage'
        joint.install(parent_workspace, stage)
        package = stage / 'src/osp_mission/osp_mission'
        shutil.copy2(HERE / 'regional_surface.py', package)
        structure = HERE.parent / '2026-11-24_aruco_print_filter/original_filter.py'
        shutil.copy2(structure, package)
        joint.edit(package / 'center_wall.py', [
            ('from osp_mission.joint_surface import JointSurface',
             'from osp_mission.regional_surface import RegionalSurface'),
            ('        self.joint = JointSurface()', '        self.joint = RegionalSurface()'),
            ('        urgent_wall = reserved is not None and elapsed-self.survey.last_wall >= 150.',
             """        urgent_wall = (reserved is not None and elapsed-self.survey.last_wall >= 150.
                       and not self.joint.can_continue(elapsed))"""),
            ('        if not crowded:\n            combined = self.joint.choose(grid, costmap, reachable, nav, distances, transform,\n'
             "                                         self.survey, candidates['wall'], fresh or candidates['interior'],\n"
             '                                         reserved, elapsed, duration)',
             """        if not crowded:
            frontier = discovery_frontier(choices, costmap, nav, veto)
            combined = self.joint.choose(grid, reachable, distances, transform, self.survey,
                                         candidates['wall'], fresh or candidates['interior'],
                                         reserved, elapsed, duration, frontier=frontier)"""),
            ("                self.reason = 'continue_local_surface_group' if combined['joint_continuation'] else 'joint_camera_gain_per_route'",
             """                self.reason = ('discover_beyond_current_region' if combined['r32_kind'] == 'interior_frontier' else
                               'continue_local_surface_group' if combined['joint_continuation'] else
                               'regional_camera_gain_per_route')"""),
            ("            self.history.append((self.phase,anchor,normal,elapsed))",
             "            kind = 'wall' if goal.get('r32_kind') == 'wall_surface' else 'interior'\n"
             "            self.history.append((kind,anchor,normal,elapsed))"),
            ("                                    self.weak_clock,duration-elapsed)",
             "                                    self.weak_clock,duration-elapsed, self.survey.camera, self.survey.reference_m)"),
        ])
        joint.edit(package / 'phased_hybrid.py', [
            ("                   joint_next_key=goal.get('joint_next_key'), joint_continuation=goal.get('joint_continuation'),",
             """                   region_anchor=goal.get('region_anchor'), region_face=goal.get('region_face'),
                   joint_continuation=goal.get('joint_continuation'),"""),
        ])
        # An unidentified cue can inspect locally, but cannot keep reversing
        # the exploration route. Named marker approach remains in charge.
        joint.edit(package / 'hybrid.py', [
            ("                score=distances[cell]*grid.resolution + abs(standoff-1.2)",
             """                walk = distances[cell] * grid.resolution
                if walk > 2.4 or any(np.linalg.norm(r['point']-p)<.75 for p,_ in self.used):
                    continue
                score=walk + abs(standoff-1.2)"""),
            ("if now-r[1]<90]", "if now-r[1]<180]"),
        ])
        joint.edit(package / 'small_wall_cues.py', [
            ('import numpy as np', 'import numpy as np\n\nfrom osp_mission.original_filter import looks_like_original'),
            ('            hits.append((u,v,short,named))', '            hits.append((u,v,short,named or looks_like_original(gray, q)))'),
        ])
        joint.edit(package / 'detector.py', [
            ('bearings=[(camera[2]-math.atan2(u-cal.k[2],cal.k[0]),size) for u,_,size,_ in small]',
             'bearings=[(camera[2]-math.atan2(u-cal.k[2],cal.k[0]),size,verified) for u,_,size,verified in small]'),
        ])
        joint.edit(package / 'mission.py', [
            ("                for angle,size in cue['bearings'][:8]:\n"
             "                    self.hybrid.schedule.weak.observe(float(cue['stamp']),now,origin,float(angle),float(size))",
             """                for bearing in cue['bearings'][:8]:
                    angle, size = map(float, bearing[:2])
                    structured = len(bearing) > 2 and bearing[2] is True
                    self.hybrid.schedule.weak.observe(float(cue['stamp']),now,origin,angle,size,structured)"""),
        ])
        joint.edit(package / 'wall_bearings.py', [
            ('def observe(self, stamp, now, origin, bearing, size):',
             'def observe(self, stamp, now, origin, bearing, size, structured=False):'),
            ('            _,p,a,s=t.samples[-1]', '            _,p,a,s,_=t.samples[-1]'),
            ('t.samples.append((stamp,tuple(origin),bearing,size))',
             't.samples.append((stamp,tuple(origin),bearing,size,structured))'),
            ('def choose(self, walls, grid, transform, known, now, time_left):',
             'def choose(self, walls, grid, transform, known, now, time_left, camera, marker_m):'),
            ('        if not walls or time_left<8 or len(self.tried)>=2:',
             '        if not walls or time_left<8 or len(self.tried)>=2 or camera is None or marker_m<=0:'),
            ('            for _,origin,angle,_ in rays:', '            for _,origin,angle,pixels,_ in rays:'),
            ('                ids &= (delta@axis>0)&(length>=1.3)&(length<=25)&(lateral<.22+.018*length)',
             """                maximum_range = 1.25 * camera[0] * marker_m / pixels
                ids &= (delta@axis>0)&(length>=.9)&(length<=maximum_range)&(lateral<.22+.018*length)"""),
            ('            stronger = any(sum(size >= 8. for _,_,_,size in t.samples) >= 2 for t in tracks)',
             '            stronger = any(sum(structured for *_,structured in t.samples) >= 2 for t in tracks)'),
            ('            if not stronger and not independent:', '            if not stronger and (not independent or walls[i][0] > 2.4):'),
        ])
        joint.edit(package / 'sensor_model.py', [
            ("    if width not in (250,320,400,480,640) or height!=width or fps not in (5.,10.,15.):\n"
             "        raise ValueError('Expected a square 250/320/400/480/640 camera at 5/10/15 FPS')",
             """    square = width in (250, 320, 400, 480, 640) and height == width
    if (not square and (width, height) != (640, 480)) or fps not in (5., 10., 15.):
        raise ValueError('Expected an installed square profile or 640×480 at 5/10/15 FPS')"""),
        ])
        names = ('center_wall.py', 'phased_hybrid.py', 'regional_surface.py', 'hybrid.py',
                 'small_wall_cues.py', 'detector.py', 'mission.py', 'wall_bearings.py', 'original_filter.py', 'sensor_model.py')
        manifest = dict(version='1.5.2.15-regional-survey',
                        parent_sha256=joint.digest(stage / 'focus_continuity_manifest.json'),
                        sources={name: joint.digest(package / name) for name in names})
        (stage / 'regional_survey_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('parent_workspace', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.parent_workspace, args.destination)
