"""Stage frame-invariant, confirmed-ID near-range witnesses in a new runtime."""
import argparse
from pathlib import Path
import shutil

ROOT=Path(__file__).resolve().parents[1]


def replace_once(code,old,new):
    if code.count(old)!=1:raise ValueError(f'Review changed patch anchor:{old[:100]}')
    return code.replace(old,new)


def approach_code(code):
    code=replace_once(code,'        self.positions = {}',
        '        self.positions = {}\n        self.range_witnesses = {}\n        self.witness_runs = {}\n        self.confirmed_ids = set()')
    code=replace_once(code,"    def observe(self, evidence, robot, now, frame='map'):",
        "    def observe(self, evidence, robot, now, frame='map', confirmed=False, odom_robot=None):")
    anchor="        self.hints[ident] = dict(point=point, observer=robot, stamp=stamp)"
    code=replace_once(code,anchor,anchor+'''
        # A physical close-range witness uses odom-relative sensor geometry,
        # not cached map coordinates that may move during SLAM correction.
        readable=(confirmed and not evidence.get('provisional',False)
                  and float(evidence.get('min_edge_pixels',0))>=18.)
        odom_point=np.asarray(evidence.get('odom_point',[])[:2],dtype=float)
        base=np.asarray(odom_robot,dtype=float) if odom_robot is not None else np.zeros(0)
        depth=np.asarray(evidence.get('camera_point',[]),dtype=float)
        laser=np.asarray(evidence.get('lidar_camera_point',[]),dtype=float)
        valid=(readable and odom_point.shape==(2,) and base.shape==(2,)
               and depth.shape==(3,) and laser.shape==(3,)
               and np.isfinite([*odom_point,*base,*depth,*laser]).all()
               and np.linalg.norm(depth-laser)<=.10)
        near_distance=float(np.linalg.norm(odom_point-base)) if valid else math.inf
        if valid and near_distance<=1.40:
            rows=self.witness_runs.setdefault(ident,[])
            if rows and (stamp-rows[-1]['stamp']>.6
                         or np.linalg.norm(odom_point-np.asarray(rows[-1]['point_odom']))>.25):
                rows.clear()
            if not rows or stamp-rows[-1]['stamp']>=.15-1e-9:
                rows.append(dict(stamp=stamp,distance_m=near_distance,
                                 robot_odom=base.tolist(),point_odom=odom_point.tolist()))
                del rows[:-3]
            if len(rows)==3:
                self.range_witnesses[ident]=dict(source='three_confirmed_readable_near_odom_observations',
                    conservative_radius_m=1.40,observations=[dict(r) for r in rows])
        else:
            self.witness_runs.pop(ident,None)
''')
    code=replace_once(code,"    def verified(self, ident, target=None):\n        evidence = self.positions.get(ident)",
        "    def verified(self, ident, target=None):\n        if ident in self.range_witnesses:return True\n        evidence = self.positions.get(ident)")
    code=replace_once(code,"    def complete(self, targets):",'''    def reconcile_targets(self, targets):
        current=set(targets)
        for ident in self.confirmed_ids-current:
            for mapping in (self.positions,self.range_witnesses,self.witness_runs,self.hints,self.near,self.last_attempt):
                mapping.pop(ident,None)
        self.confirmed_ids=current

    def complete(self, targets):''')
    code=replace_once(code,"                    verified_ids=sorted(i for i in self.positions if self.verified(i)))",
        "                    range_witness_evidence=self.range_witnesses,\n                    verified_ids=sorted(i for i in self.positions.keys()|self.range_witnesses.keys() if self.verified(i)))")
    return code


def mission_code(code):
    code=replace_once(code,"'map_frame':'map','base_frame':'base_link','scan_topic':'scan',",
        "'map_frame':'map','base_frame':'base_link','odom_frame':'odom','scan_topic':'scan',")
    old="""            self.approach.observe(evidence,(tr.transform.translation.x,tr.transform.translation.y),
                self.get_clock().now().nanoseconds/1e9,self.p('map_frame'))"""
    new="""            odom_robot=None
            try:
                odom=self.tf.lookup_transform(self.p('odom_frame'),self.p('base_frame'),Time(seconds=stamp)).transform.translation
                odom_robot=(odom.x,odom.y)
            except Exception:pass
            self.approach.observe(evidence,(tr.transform.translation.x,tr.transform.translation.y),
                self.get_clock().now().nanoseconds/1e9,self.p('map_frame'),
                confirmed=int(evidence['id']) in self.targets,odom_robot=odom_robot)"""
    code=replace_once(code,old,new)
    old="        self.targets={a.id:(a.pose.position.x,a.pose.position.y) for a in m.markers if a.ns=='point'}"
    code=replace_once(code,old,old+"\n        if getattr(self,'approach',None) is not None:self.approach.reconcile_targets(self.targets)")
    return code


def prepare(source,destination):
    if destination.exists():raise FileExistsError('Use a new immutable runtime')
    package=destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission',package,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    for name,patch in [('marker_approach.py',approach_code),('mission.py',mission_code)]:
        path=package/'osp_mission'/name;path.write_text(patch(path.read_text()))
    shutil.copy2(ROOT/'scripts/test_approach_witness_runtime.py',package/'test/test_approach_witness_runtime.py')
    return destination


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--destination',type=Path,required=True)
    args=parser.parse_args();print(prepare(args.source.resolve(),args.destination.resolve()))
