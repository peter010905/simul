"""Create a NEW world-shape runtime; optional exploration revisit memory."""
import argparse
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]


def replace_once(code, old, new):
    if code.count(old) != 1:
        raise ValueError(f'Patch anchor changed: {old[:90]}')
    return code.replace(old, new)


def world_code(code):
    start = code.index("    box('floor', 10.075")
    end = code.index("    for j,o in enumerate(cfg.get('obstacles',[])):")
    legacy = code[start:end]
    custom = '''    if cfg.get('arena'):
        arena=cfg['arena'];w,h=arena['width_m'],arena['height_m']
        box('floor',w/2,h/2,-.06,w,h,.12,'.24 .28 .31')
        for j,wall in enumerate(arena['walls']):
            box(f'arena_wall{j}',wall['x'],wall['y'],wall['height']/2,
                wall['sx'],wall['sy'],wall['height'],'.75 .77 .78',yaw=wall['yaw'])
    else:
'''
    code = code[:start]+custom+''.join('    '+line+'\n' for line in legacy.splitlines())+code[end:]
    start = code.index('    for inset in [0., .845]:')
    end = code.index('    dictionary=cv2.aruco')
    legacy = code[start:end]
    code = code[:start]+"    if not cfg.get('arena'):\n"+''.join('    '+line+'\n' for line in legacy.splitlines())+code[end:]
    anchor = "    paint=ET.SubElement(scene,'model',name='floor_markings')"
    replacement = '''    if cfg.get('arena'):
        from osp_mission.space_geometry import write_floor_mesh
        mesh=write_floor_mesh(cfg['arena'],out)
        geometry=scene.find("model[@name='floor']/link/visual/geometry")
        geometry.clear()
        ET.SubElement(ET.SubElement(geometry,'mesh'),'uri').text=str(mesh)
        observer=scene.find("model[@name='mission_observer']/pose")
        if observer is not None:
            w,h=cfg['arena']['width_m'],cfg['arena']['height_m'];span=max(w,h)
            observer.text=f'{w/2} {h/2-.58*span} {.95*span} 0 1.022 1.57079633'
    paint=ET.SubElement(scene,'model',name='floor_markings')'''
    return replace_once(code, anchor, replacement)


def memory_code(code):
    code = replace_once(code, 'from osp_mission.marker_focus import MarkerFocus',
        'from osp_mission.marker_focus import MarkerFocus\nfrom osp_mission.exploration_memory import ExplorationMemory')
    code = replace_once(code, '        self.marker_focus=MarkerFocus()',
        '        self.marker_focus=MarkerFocus()\n        self.exploration_memory=ExplorationMemory()')
    code = replace_once(code, "        if self.state!='running': return\n        if getattr(self,'approach',None) is not None and self.targets:",
        """        if self.state!='running': return
        try:
            visit=self.tf.lookup_transform('odom',self.p('base_frame'),Time()).transform.translation
            self.exploration_memory.observe(visit.x,visit.y,self.now())
        except Exception:pass
        if getattr(self,'approach',None) is not None and self.targets:""")
    code = replace_once(code, "        if self.kind=='nav':\n            self.excluded.append(self.destination); self.scan_after=result.status==4",
        "        if self.kind=='nav':\n            if getattr(self,'exploration_memory',None) is not None:self.exploration_memory.completed(result.status==4,self.now())\n            self.excluded.append(self.destination); self.scan_after=result.status==4")
    anchor = "        selected=(choose_before_deadline(choices,self.weights,self.p('duration')-(self.now()-self.started))"
    code = replace_once(code, anchor, """        memory_transform=None
        if self.edge_mode:
            try:
                mt=self.tf.lookup_transform('odom',self.p('map_frame'),Time()).transform
                mq=mt.rotation
                memory_transform=(mt.translation.x,mt.translation.y,math.atan2(2*(mq.w*mq.z+mq.x*mq.y),1-2*(mq.y*mq.y+mq.z*mq.z)))
                choices=self.exploration_memory.filter(choices,memory_transform,self.now())
            except Exception as exc:self.get_logger().debug(f'Exploration memory TF unavailable: {exc}')
"""+anchor)
    anchor = "        self.empty_plans=0\n        self.next_scan_angle=selected.get('scan_angle',2*math.pi/3)"
    return replace_once(code, anchor, """        self.empty_plans=0
        if memory_transform is not None:
            self.exploration_memory.selected(selected,memory_transform)
            try:
                with (Path(self.p('output_dir'))/'exploration_memory.jsonl').open('a') as stream:
                    stream.write(json.dumps(dict(elapsed_seconds=self.now()-self.started,
                        selected=[selected['x'],selected['y'],selected['yaw']],
                        marker_override='approach_id' in selected or 'focus_id' in selected,
                        **self.exploration_memory.report()))+'\\n')
            except OSError as exc:self.get_logger().warning(str(exc))
        self.next_scan_angle=selected.get('scan_angle',2*math.pi/3)""")


def prepare(source, destination, revisit_memory=True):
    if destination.exists():
        raise FileExistsError('Use a new runtime; existing runtimes are frozen.')
    package = destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission', package, ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache'))
    world = package/'osp_mission/world.py'
    world.write_text(world_code(world.read_text()))
    for name in ('space_geometry.py', 'space_scenarios.py', 'exploration_memory.py', 'marker_sizes.py'):
        shutil.copy2(ROOT/'src/osp_mission/osp_mission'/name, package/'osp_mission'/name)
    if revisit_memory:
        mission = package/'osp_mission/mission.py'
        mission.write_text(memory_code(mission.read_text()))
    shutil.copy2(ROOT/'scripts/test_spatial_runtime.py', package/'test/test_spatial_runtime.py')
    return destination


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--destination', required=True, type=Path)
    parser.add_argument('--no-revisit-memory', action='store_true')
    args = parser.parse_args()
    print(prepare(args.source.resolve(), args.destination.resolve(), not args.no_revisit_memory))
