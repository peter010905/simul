"""Install Mixed v1.5.2.12 with a bounded look at unseen box faces in transit."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
PARENT_MISSION = '6c43c48ed2599b0d468bdededc5e2f9f985ec08158150676c77e032fc078a821'


def replace(source, old, new):
    if source.count(old) != 1:
        raise ValueError('Unexpected v1.5.2.11 mission source: '+old)
    return source.replace(old, new, 1)


def install(release, destination):
    destination = destination.resolve()
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    parent = HERE.parent/'2026-11-23_v152_weak_wall_priority/install.py'
    with tempfile.TemporaryDirectory(dir=destination.parent) as temp:
        stage = Path(temp)/'stage'
        subprocess.run([sys.executable, str(parent), str(release.resolve()), str(stage)], check=True)
        package = stage/'src/osp_mission/osp_mission'
        shutil.copy2(HERE/'box_pass.py', package)
        mission = package/'mission.py'
        if hashlib.sha256(mission.read_bytes()).hexdigest() != PARENT_MISSION:
            raise ValueError('Unexpected v1.5.2.11 mission digest')
        source = mission.read_text()
        source = replace(source, "                  (flow is None or flow.allow_mid_route_wall_inspection)):self.near_pass.request()",
                         "                  not (self.contact_pending or self.recovery_pending or guard.phase or bearing_fresh)\n"
                         "                  and self.spin.server_is_ready()):self.near_pass.request()")
        source = replace(source, '            self.near_pass.inspectable_route=False  # The phase scheduler owns surface observations.',
                         "            self.near_pass.inspectable_route = (not self.near_pass.protected_navigation\n"
                         "                and selected.get('r32_kind') in ('interior_surface', 'interior_frontier', 'unvisited_region')\n"
                         "                and not selected.get('survey_region'))")
        for reason, guard in (('readable_marker', "self.cancel_reason == 'readable_marker' and result.status in (4,5)"),
                              ('registered_marker', "self.cancel_reason == 'registered_marker' and self.kind in ('nav','spin') and result.status in (4,5)")):
            line = f'        if {guard}:\n'
            source = replace(source, line, line+f"            self.near_pass.abort('{reason}')\n")
        compile(source, str(mission), 'exec')
        mission.write_text(source)
        runtime = package/'near_pass_runtime.py'
        source = runtime.read_text()
        source = replace(source, 'from osp_mission.policy import wrap',
                         'from osp_mission.policy import wrap\nfrom osp_mission.box_pass import BoxPassInspection')
        source = replace(source, '        self.protected_navigation=False',
                         '        self.protected_navigation=False\n        self.box_policy = BoxPassInspection()')
        source = replace(source, '        self.policy.reserve(choice, now)\n        self.task = dict(',
                         "        policy = self.box_policy if choice.get('source') == 'box_pass' else self.policy\n"
                         '        policy.reserve(choice, now)\n        self.task = dict(policy=policy, ')
        source = replace(source, "turn_limit=choice.get('turn_limit', self.policy.maximum_turn_rad+.05),",
                         "turn_limit=choice.get('turn_limit', policy.maximum_turn_rad+.05),")
        source = replace(source, "limit=min(self.task.get('time_limit',10.), max(0.,self.policy.maximum_total_sim_s-self.policy.spent))",
                         "policy = self.task['policy']\n"
                         "        limit=min(self.task.get('time_limit',10.), max(0.,policy.maximum_total_sim_s-policy.spent))")
        source = replace(source, 'self.policy.finish(owner.now(),reason);self.task=None',
                         "task['policy'].finish(owner.now(),reason);self.task=None")
        source = replace(source, "self.event('finished',reason=reason,resumed_navigation=retain_navigation)",
                         "self.event('finished',reason=reason,resumed_navigation=retain_navigation,source=task['source'])")
        source = replace(source, "        if owner.map is None or calibration is None:return",
                         "        if (owner.map is None or calibration is None or self.print_hazard\n"
                         "                or not 0 <= owner.now()-self.hazard_received <= .85):return")
        source = replace(source, 'choice=self.policy.choose(data,', 'choice=self.box_policy.choose(data,')
        source = replace(source, "if (self.task.get('source') == 'edge_print' and",
                         "if (self.task.get('source') in ('edge_print', 'box_pass') and")
        source = replace(source, "if task.get('source') == 'edge_print':",
                         "if task.get('source') in ('edge_print', 'box_pass'):")
        compile(source, str(runtime), 'exec')
        runtime.write_text(source)
        manifest = dict(version='1.5.2.12-box-pass', parent='1.5.2.11-weak-wall-priority',
                        sources={name: hashlib.sha256((package/name).read_bytes()).hexdigest()
                                 for name in ('mission.py', 'near_pass_runtime.py', 'box_pass.py')})
        (stage/'box_pass_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.release, args.destination)
