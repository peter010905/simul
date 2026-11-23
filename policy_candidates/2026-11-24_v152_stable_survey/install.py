"""Install stable region exploration in a separate Focus-derived workspace."""
import argparse
import importlib.util
import json
import shutil
import tempfile
from pathlib import Path


HERE = Path(__file__).resolve().parent
REGIONAL = HERE.parent / '2026-11-24_v152_regional_survey/install.py'
spec = importlib.util.spec_from_file_location('regional_survey_installer', REGIONAL)
regional = importlib.util.module_from_spec(spec)
spec.loader.exec_module(regional)
edit = regional.joint.edit


def install(parent_workspace, destination):
    destination = destination.resolve()
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temp:
        stage = Path(temp) / 'stage'
        regional.install(parent_workspace, stage)
        package = stage / 'src/osp_mission/osp_mission'
        shutil.copy2(HERE / 'action_priority.py', package)
        edit(package / 'mission.py', [
            ('from osp_mission.exploration_memory import ExplorationMemory',
             'from osp_mission.exploration_memory import ExplorationMemory\n'
             'from osp_mission.action_priority import SENSOR_REASONS, priority'),
            ("self.cancel_reason='';self.undock_diagnostics={};", "self.cancel_reason='';self.cancel_started=None;self.undock_diagnostics={};"),
            ("self.kind=kind;self.cancel_reason=''", "self.kind=kind;self.cancel_reason='';self.cancel_started=None"),
            ('client.send_goal_async(goal,**feedback).add_done_callback(self.accepted)',
             'client.send_goal_async(goal,**feedback).add_done_callback(lambda result:self.accepted(result,generation))'),
            ("""        if self.cancel_reason:return
        self.cancel_reason=reason
        self.get_logger().warning(f'Cancel {self.kind}: {reason}')
        if self.goal_handle:self.goal_handle.cancel_goal_async()
    def accepted(self,future):
        self.pending=False""",
             """        previous = self.cancel_reason
        if priority(reason) <= priority(previous):
            return
        self.cancel_reason = reason
        self.get_logger().warning(f'Cancel {self.kind}: {reason}')
        if not previous:
            self.cancel_started = self.now()
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
    def accepted(self,future,generation=None):
        if generation is not None and generation != self.action_generation:
            try:
                old_handle = future.result()
                if old_handle.accepted:
                    old_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().debug(f'Obsolete action response: {exc}')
            return
        self.pending=False"""),
            ('handle.get_result_async().add_done_callback(self.completed)\n    def completed(self,future):\n        self.goal_handle=None',
             """handle.get_result_async().add_done_callback(lambda result:self.completed(result,generation))
    def completed(self,future,generation=None):
        if generation is not None and generation != self.action_generation:
            return
        self.goal_handle=None
        self.cancel_started=None"""),
            ("        if getattr(self,'cancel_reason','')=='inspection_due' and result.status==5:\n            self.scan_after=True;return\n", ''),
            ("        if self.cancel_reason == 'readable_marker' and result.status in (4,5):",
             """        if self.cancel_reason == 'inspection_due' and result.status == 5:
            self.scan_after = True
            return
        if self.cancel_reason == 'readable_marker' and result.status in (4,5):"""),
            ("""        sensor_reasons={'processed_camera_stale','scan_stale','map_unavailable','map_to_base_tf_unavailable','nav2_unavailable','drive_unavailable'}
        if result.status==5 and getattr(self,'cancel_reason','') in sensor_reasons""",
             """        if result.status==5 and getattr(self,'cancel_reason','') in SENSOR_REASONS"""),
            ("        # Create 3 owns the undock motion and its hazard checks. Its action",
             """        if (self.pending or self.goal_handle) and self.cancel_started is not None:
            if self.now()-self.cancel_started >= 8.:
                self.zero()
                self.finish('action_cancel_unconfirmed')
                return
        # Create 3 owns the undock motion and its hazard checks. Its action"""),
        ])
        edit(package / 'near_pass_runtime.py', [
            ("        if not owner.cancel_reason:owner.cancel_action('near_pass_time_limit')\n"
             "        if owner.goal_handle is not None:owner.goal_handle.cancel_goal_async()",
             "        owner.cancel_action('near_pass_time_limit')"),
        ])
        edit(package / 'hybrid.py', [
            ('        if not fresh_print and not probe_box:\n            return False',
             """        if not fresh_print and not probe_box:
            return False
        # Anonymous interruption is optional while the regional survey has
        # safe local work. Named marker and safety signals use their own paths.
        survey = getattr(getattr(self, 'schedule', None), 'joint', None)
        if survey is not None and survey.can_continue(now-owner.started):
            return False"""),
        ])
        edit(package / '_r32_navigation_core.py', [
            ("""        for p in np.linspace(a,b,n)[:-2]:
            r,c = self.index(*p)
            if not self.inside(r,c) or (self.cells[r,c] < 0 and not allow_unknown) or self.cells[r,c] > 25:
                return False
        return True""",
             """        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = (np.linspace(a, b, n)[:-2] - [ox, oy]).T
        row = np.floor((-s*dx + c*dy)/self.resolution).astype(np.intp)
        col = np.floor((c*dx + s*dy)/self.resolution).astype(np.intp)
        if ((row < 0) | (col < 0) | (row >= self.cells.shape[0]) | (col >= self.cells.shape[1])).any():
            return False
        cells = self.cells[row, col]
        return not ((cells > 25) | ((cells < 0) & (not allow_unknown))).any()"""),
        ])
        edit(package / 'regional_surface.py', [
            ("""        if 'region_anchor' not in goal:
            # A marker interruption can resume the same region within its""",
             """        if goal.get('r32_kind') == 'interior_frontier' and self.anchor is not None:
            self.recent_regions[region_key(self.anchor)] = elapsed
            self.anchor = None
        if 'region_anchor' not in goal:
            # A marker interruption can resume the same region within its"""),
            ("""               walls, interiors, reserved, elapsed, duration, *, frontier=None):""",
             """               walls, interiors, reserved, elapsed, duration, *, frontier=None, heading=None):"""),
            ("""            gain = sum(sorted(seen.values(), reverse=True)[:3])""",
             """            extra_wall = max((v for k, v in seen.items() if k[0] == 'wall' and k != view.key), default=0.)
            extra_object = max((v for k, v in seen.items() if k[0] != 'wall' and k != view.key), default=0.)
            gain = view.weight + .35 * (extra_wall + extra_object)"""),
            ('            score = gain / (1. + .45 * view.walk + repeat_cost)',
             """            turn = abs(math.remainder(view.goal['yaw']-heading, math.tau)) if heading is not None else 0.
            score = gain / (1. + .45 * view.walk + .10 * turn + repeat_cost)"""),
            ('local[0][0] >= .65 * score', 'local[0][0] >= .78 * score'),
            ('frontier_score > 1.15 * score', 'frontier_score > score'),
        ])
        edit(package / 'center_wall.py', [
            ('reserved, elapsed, duration, frontier=frontier)',
             'reserved, elapsed, duration, frontier=frontier, heading=pose[2])'),
            ("                    score = distances[cell] + .5*abs(stand-.67) + .3*abs(shift) + 2.*len(attempts)",
             """                    reference = self.survey.standoffs()[-1]
                    score = distances[cell] + .65*abs(stand-reference) + .3*abs(shift) + 2.*len(attempts)"""),
        ])
        changed = ('mission.py', 'action_priority.py', 'near_pass_runtime.py', 'hybrid.py',
                   '_r32_navigation_core.py', 'regional_surface.py', 'center_wall.py')
        digest = regional.joint.digest
        manifest = dict(version='1.5.2.16-stable-survey',
                        parent_sha256=digest(stage / 'regional_survey_manifest.json'),
                        sources={name: digest(package / name) for name in changed})
        (stage / 'stable_survey_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('parent_workspace', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.parent_workspace, args.destination)
