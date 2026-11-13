"""Stage explicit marker-directed looks ahead of generic exploration sweeps."""
import argparse
from pathlib import Path
import shutil

from prepare_approach_witness_runtime import replace_once

ROOT = Path(__file__).resolve().parents[1]


def prepare(source, destination):
    if destination.exists():
        raise FileExistsError('Use a new immutable marker-directed stage')
    package = destination / 'src/osp_mission'
    shutil.copytree(source / 'src/osp_mission', package,
                    ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache'))
    shutil.copy2(ROOT/'scripts/candidate_marker_directed_look.py', package/'osp_mission/marker_directed_look.py')
    shutil.copy2(ROOT/'scripts/test_marker_directed_runtime.py', package/'test/test_marker_directed_runtime.py')
    path = package/'osp_mission/mission.py'
    code = path.read_text()
    patches = [
        ('from osp_mission.marker_focus import MarkerFocus',
         'from osp_mission.marker_focus import MarkerFocus\nfrom osp_mission.marker_directed_look import MarkerDirectedLook'),
        ('        self.marker_focus=MarkerFocus()',
         '        self.marker_focus=MarkerFocus()\n        self.marker_look=MarkerDirectedLook(self)'),
        ('    def observation_turn(self):\n',
         "    def observation_turn(self):\n        look=getattr(self,'marker_look',None)\n        directed=look.consume() if look is not None else None\n        if directed is not None:return directed\n"),
        ("            if getattr(self,'kind','')=='undock':return",
         "            if getattr(self,'marker_look',None) is not None:self.marker_look.clear('bump_contact')\n            if getattr(self,'kind','')=='undock':return"),
        ("            self.get_logger().error(str(exc))\n            if self.kind=='undock'",
         "            self.get_logger().error(str(exc))\n            if getattr(self,'marker_look',None) is not None:self.marker_look.clear('action_request_error')\n            if self.kind=='undock'"),
        ("        if not handle.accepted:\n            self.contact_pending=False",
         "        if not handle.accepted:\n            self.contact_pending=False\n            if getattr(self,'marker_look',None) is not None:self.marker_look.clear('action_rejected')"),
        ("        except Exception as exc:\n            if self.kind=='undock':\n                self.undock_diagnostics=",
         "        except Exception as exc:\n            if getattr(self,'marker_look',None) is not None:self.marker_look.clear('action_result_error')\n            if self.kind=='undock':\n                self.undock_diagnostics="),
        ("        if getattr(self,'contact_pending',False):\n",
         "        if getattr(self,'contact_pending',False):\n            if getattr(self,'marker_look',None) is not None:self.marker_look.clear('bump_contact')\n"),
        ("        if result.status==5 and getattr(self,'cancel_reason','')=='marker_hint' and self.kind=='nav':\n",
         "        if result.status==5 and getattr(self,'cancel_reason','')=='marker_hint' and self.kind=='nav':\n            if getattr(self,'marker_look',None) is not None:self.marker_look.clear('new_marker_hint')\n"),
        ("            self.excluded.append(self.destination); self.scan_after=result.status==4",
         "            self.excluded.append(self.destination); self.scan_after=result.status==4\n            if result.status!=4 and getattr(self,'marker_look',None) is not None:self.marker_look.clear('navigation_failed')"),
        ("    def finish(self,reason):\n        if self.state!='running': return\n",
         "    def finish(self,reason):\n        if self.state!='running': return\n        if getattr(self,'marker_look',None) is not None:self.marker_look.clear('mission_finished')\n"),
        ("        report={'graph_landmark_update':", "        report={'marker_directed_look':self.marker_look.report(),'graph_landmark_update':"),
        ("        tr=self.tf.lookup_transform(self.p('map_frame'),self.p('base_frame'),Time())\n        m=self.map; data=",
         "        self.marker_look.clear('replan')\n        tr=self.tf.lookup_transform(self.p('map_frame'),self.p('base_frame'),Time())\n        m=self.map; data="),
        ("        self.next_scan_angle=selected.get('scan_angle',2*math.pi/3)",
         "        self.marker_look.select(selected,memory_transform)\n        self.next_scan_angle=selected.get('scan_angle',2*math.pi/3)"),
    ]
    for before, after in patches:
        code = replace_once(code, before, after)
    path.write_text(code)
    return destination


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('source','destination'):
        parser.add_argument('--'+name,type=Path,required=True)
    args = parser.parse_args()
    print(prepare(args.source.resolve(),args.destination.resolve()))
