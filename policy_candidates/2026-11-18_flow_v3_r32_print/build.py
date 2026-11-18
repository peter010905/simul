"""Stage the optional hybrid from the released Flow v3 and a locally authorized R32 core."""
import argparse
import hashlib
from pathlib import Path
import shutil

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]
CAND=HERE.parent
REF_HASH='9eaa75ca17c81e0a4fa3700933a21cdda4400c98e0c31681fb12ccd6d32bd90b'


def patch(path,old,new):
    text=path.read_text()
    if text.count(old)!=1:
        raise ValueError(f'Unexpected parent signature in {path.name}: {old[:60]}')
    path.write_text(text.replace(old,new))


def stage(dest,core):
    if hashlib.sha256(core.read_bytes()).hexdigest()!=REF_HASH:
        raise ValueError('Expected src/core.py from yeonslay commit b1ed300e0052')
    dest.mkdir(parents=True,exist_ok=False)
    release=REPO/'policy_releases/2026-11-18_flow_v3_dashboard'
    for folder in ('src','applied_config','policy_releases','robot_profiles'):
        shutil.copytree(release/folder,dest/folder)
    shutil.copy(release/'build.sh',dest)
    pkg=dest/'src/osp_mission'
    mod=pkg/'osp_mission'
    shutil.copy(core,mod/'_r32_navigation_core.py')
    shutil.copy(HERE/'hybrid.py',mod)
    shutil.copy(HERE/'test_hybrid.py',pkg/'test')
    shutil.copy(CAND/'2026-11-18_flow_v3_preview_bearing/preview_geometry.py',mod/'hybrid_geometry.py')
    shutil.copy(CAND/'2026-11-18_flow_v3_visual_cue/overlay/osp_mission/visual_cue.py',mod/'hybrid_visual.py')
    path=mod/'hybrid_borders.py'
    source=(CAND/'2026-11-18_flow_v3_border_recovery/recovery.py').read_text()
    path.write_text(source[:source.index('def plane_hit')].replace('from visual_cue import','from osp_mission.hybrid_visual import'))
    shutil.copy(CAND/'2026-11-18_flow_v3_edge_glance/glance.py',mod/'hybrid_glance.py')
    passing=(CAND/'2026-11-18_flow_v3_passing_glance/passing.py').read_text()
    start=passing.index('sys.path.insert')
    passing=passing[:start]+passing[passing.index('from glance import',start):]
    (mod/'hybrid_passing.py').write_text(passing.replace('from glance import','from osp_mission.hybrid_glance import'))
    drive=mod/'r32_drive.py'
    shutil.copy(CAND/'2026-11-18_r32_drive_original18/overlay/osp_mission/r32_drive.py',drive)
    patch(drive,'self.detours = 0\n        self.diagnostic','self.detours = 0\n        self.transit = []\n        self.inspect_boxes = False\n        self.diagnostic')
    patch(drive,'allow_look=False)','allow_look=self.inspect_boxes, corridor_forward=True, transit_views=list(self.transit))')
    patch(drive,'mission.navigation_flow.failures','getattr(mission.navigation_flow,"failures",[])')
    patch(drive,"        if goal is None:", """        if goal is not None and goal[3]=='look':
            if info.get('selection_policy')!='passed_box_look':
                return None
            point=info['goal_surfaces'][0][:2]
            return dict(x=pose[0],y=pose[1],yaw=goal[2],scan_angle=0.,stationary=True,
                        distance=0.,features=np.zeros(5),r32_kind='box_look',
                        box_center=info['box_center'],box_point=point)
        if goal is None:""")
    path=mod/'detector.py'
    patch(path,'        self.candidate_pub=',"        from osp_mission.hybrid_passing import PassingGlance\n        self.print_glance=PassingGlance()\n        self.glance_pub=self.create_publisher(String,'mission/anonymous_glance',10)\n        self.print_pub=self.create_publisher(String,'mission/anonymous_print',10)\n        self.candidate_pub=")
    patch(path,'            if ids is None: return',"""            if ids is None:
                try:
                    from osp_mission.hybrid import publish_print
                    publish_print(self,msg,image)
                except Exception as exc:
                    self.get_logger().debug(f'Anonymous print projection unavailable: {exc}')
                return""")
    path=mod/'mission.py'
    patch(path,'        self.create_timer(.5,self.tick)',"""        from osp_mission.hybrid import Hybrid
        self.contact_pending=False
        self.hybrid=Hybrid() if self.edge_mode else None
        if self.hybrid is not None:
            self.create_subscription(OccupancyGrid,'global_costmap/costmap',self.hybrid.drive.update_costmap,qos_profile_sensor_data)
            self.create_subscription(String,'mission/anonymous_print',self.on_anonymous_print,10)
            self.create_subscription(String,'mission/anonymous_glance',self.on_anonymous_glance,10)
        self.create_timer(.5,self.tick)

    def on_anonymous_glance(self,msg):
        guard=self.contact_inspection
        now=self.now()
        if (self.state!='running' or self.contact_pending or self.recovery_pending or guard.phase
                or self.marker_look.pending or self.face_look.pending or self.near_pass.protected_navigation
                or (guard.bearing and now-guard.bearing[0]<=2.) or now<self.spin_blocked_until):
            return
        try:
            e=json.loads(msg.data)
            angle=float(e['angle'])
            if abs(angle)<=math.radians(6.01) and -.1<=self.get_clock().now().nanoseconds/1e9-e['stamp']<=.5:
                guard.bearing=(now,angle)
        except (TypeError,ValueError,KeyError):
            return

    def on_anonymous_print(self,msg):
        if self.state=='running':
            try:
                self.hybrid.memory.observe(json.loads(msg.data),self.get_clock().now().nanoseconds/1e9)
            except (ValueError,TypeError,KeyError):
                return""")
    patch(path,"        try:\n            save_policy_audit(", """        hybrid=getattr(self,'hybrid',None)
        if hybrid is not None:
            try:
                combined=hybrid.select(self,selected,data,viewed,(tr.transform.translation.x,tr.transform.translation.y,heading),memory_transform)
                if combined is not None and combined is not selected:
                    choices.append(combined)
                    selected=combined
            except Exception as exc:
                self.get_logger().warning(f'Hybrid proposal unavailable, using Flow choice: {exc}')
        try:
            save_policy_audit(""")
    patch(path,"        if self.goal_handle or self.pending:\n            if (guard is not None", """        if self.goal_handle or self.pending:
            hybrid=getattr(self,'hybrid',None)
            if (hybrid is not None and self.kind=='nav' and not protected and not self.cancel_reason
                    and not self.contact_pending and not self.recovery_pending and not guard.phase
                    and not bearing_fresh and now>=self.spin_blocked_until
                    and not getattr(self.near_pass,'protected_navigation',False)
                    and hybrid.should_interrupt(self)):
                self.cancel_action('marker_hint');self.zero();return
            if (guard is not None""")
    patch(path,'        self.empty_plans=0\n        if memory_transform',"""        self.empty_plans=0
        if hybrid is not None:
            hybrid.dispatched(self,selected)
        if memory_transform""")
    patch(path,"self.near_pass.protected_navigation='approach_id' in selected or 'focus_id' in selected",
          "self.near_pass.protected_navigation=any(k in selected for k in ('approach_id','focus_id','print_point','box_point'))")
    path=mod/'marker_directed_look.py'
    patch(path,"            kind = 'focus'\n        else:", """            kind = 'focus'
        elif 'print_point' in selected:
            ident = None
            point = selected['print_point']
            kind = 'anonymous_print'
        elif 'box_point' in selected:
            ident = None
            point = selected['box_point']
            kind = 'passed_box'
        else:""")
    patch(path,'dict(id=int(ident), kind=kind','dict(id=None if ident is None else int(ident), kind=kind')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('workspace',type=Path)
    parser.add_argument('--reference-core',type=Path,required=True)
    args=parser.parse_args()
    stage(args.workspace,args.reference_core)
