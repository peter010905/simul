"""Stage short nearby-face inspections without changing ID or approach credit."""
import argparse
from pathlib import Path
import shutil

from prepare_approach_witness_runtime import replace_once

ROOT=Path(__file__).resolve().parents[1]


def prepare(source,destination):
    if destination.exists():raise FileExistsError('Use a new immutable runtime stage')
    package=destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission',package,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    for source_name,module in [('candidate_near_pass_inspection.py','near_pass_inspection.py'),
                               ('candidate_near_pass_runtime.py','near_pass_runtime.py')]:
        shutil.copy2(ROOT/'scripts'/source_name,package/'osp_mission'/module)
    for filename in ['test_near_pass_inspection_runtime.py','test_near_pass_controller_runtime.py']:
        shutil.copy2(ROOT/'scripts'/filename,package/'test'/filename)
    path=package/'osp_mission/mission.py';code=path.read_text()
    code=replace_once(code,'from osp_mission.exploration_memory import ExplorationMemory',
        'from osp_mission.exploration_memory import ExplorationMemory\nfrom osp_mission.near_pass_runtime import NearPassRuntime')
    code=replace_once(code,'        self.exploration_memory=ExplorationMemory()',
        '        self.exploration_memory=ExplorationMemory()\n        self.near_pass=NearPassRuntime(self)')
    code=replace_once(code,'            self.contact_pending=True;self.resume_action=None',
        "            if getattr(self,'near_pass',None) is not None:self.near_pass.abort('bump_contact')\n            self.contact_pending=True;self.resume_action=None")
    code=replace_once(code,'        if not handle.accepted:\n            self.contact_pending=False',
        "        if not handle.accepted:\n            self.contact_pending=False\n            if getattr(self,'near_pass',None) is not None and self.near_pass.rejected(self.kind):return")
    code=replace_once(code,"        sensor_reasons={'processed_camera_stale'",
        "        if getattr(self,'near_pass',None) is not None and self.near_pass.completed(result.status,self.kind,getattr(self,'cancel_reason','')):return\n"
        "        sensor_reasons={'processed_camera_stale'")
    code=replace_once(code,"    def finish(self,reason):\n        if self.state!='running': return\n",
        "    def finish(self,reason):\n        if self.state!='running': return\n        if getattr(self,'near_pass',None) is not None:self.near_pass.abort('mission_finished')\n")
    code=replace_once(code,"report={'marker_focus_profile':dict(",
        "report={'near_pass_inspection':self.near_pass.policy.report(),'marker_focus_profile':dict(")
    code=replace_once(code,"        if self.goal_handle or self.pending:\n            if self.now()-self.action_start>65:self.cancel_action('action_timeout')\n            return",
        "        if getattr(self,'near_pass',None) is not None:self.near_pass.poll()\n"
        "        if self.goal_handle or self.pending:\n"
        "            if self.now()-self.action_start>65:self.cancel_action('action_timeout')\n"
        "            elif getattr(self,'near_pass',None) is not None:self.near_pass.request()\n"
        "            return\n"
        "        if getattr(self,'near_pass',None) is not None and self.near_pass.tick():return")
    code=replace_once(code,"        self.next_scan_angle=selected.get('scan_angle',2*math.pi/3)",
        "        if getattr(self,'near_pass',None) is not None:self.near_pass.protected_navigation='approach_id' in selected or 'focus_id' in selected\n"
        "        self.next_scan_angle=selected.get('scan_angle',2*math.pi/3)")
    path.write_text(code)
    return destination


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('source','destination'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();print(prepare(args.source.resolve(),args.destination.resolve()))
