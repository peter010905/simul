"""Opt-in Mixed variant. The inherited mission is the only Nav2 action owner."""
import json
import math
from pathlib import Path
import numpy as np
from rclpy.time import Time
from osp_mission import dock_geometry
from osp_mission.phased_hybrid import PhasedHybrid
from osp_mission.wall_route import yaw
from osp_mission.nbv_faces import MultiView
from osp_mission.nbv_rgb import RGBMemory

HOLD_S = 4.
HYSTERESIS = .18

class MarkerNBV(PhasedHybrid):
    def __init__(self):
        super().__init__()
        self.schedule = MultiView()
        self.memory = RGBMemory()
        self.current = {}
        self.changed_at = -math.inf
        self.checked_revision = -1
        self.registered = frozenset()
        self.confirm_pending = False

    def read_targets(self,owner):
        ids = frozenset(owner.targets)
        if ids-self.registered:
            self.memory.changed('registered')
            self.confirm_pending=True
        self.registered=ids

    def interrupt_allowed(self,owner):
        return owner.now()-self.changed_at>=HOLD_S

    def should_interrupt(self,owner):
        now=owner.now()
        stamp=owner.get_clock().now().nanoseconds/1e9
        self.read_targets(owner)
        self.memory.expire(stamp)
        wall_due = now>=self.next_box_probe
        if not self.interrupt_allowed(owner) or now<self.next_check:
            return False
        if self.checked_revision==self.memory.revision and not wall_due and not self.confirm_pending:
            return False
        self.next_check=now+1.
        if wall_due:
            self.next_box_probe=now+3.
        try:
            pose=owner.tf.lookup_transform(owner.p('map_frame'),owner.p('base_frame'),Time()).transform
            mt=owner.tf.lookup_transform('odom',owner.p('map_frame'),Time()).transform
        except Exception as exc:
            owner.get_logger().debug(f'NBV re-evaluation awaits TF: {exc}')
            return False
        self.checked_revision=self.memory.revision
        data=np.asarray(owner.map.data).reshape(owner.map.info.height,owner.map.info.width).copy()
        dock=owner.known_dock_pose(owner.p('map_frame'))
        if dock is not None:
            info=owner.map.info
            dock_geometry.mark(data,info.resolution,(info.origin.position.x,info.origin.position.y),dock)
        transform=(mt.translation.x,mt.translation.y,yaw(mt.rotation))
        self.memory.reconcile(owner.targets,transform)
        if self.confirm_pending:
            # A newly registered ID must be considered by the unchanged close-approach layer.
            self.confirm_pending=False
            return True
        other=self.select(owner,None,data,None,(pose.translation.x,pose.translation.y,yaw(pose.rotation)),
                          transform,print_only=True,box_probe=wall_due)
        track = next((t for t in self.memory.tracks if t.id==self.current.get('nbv_rgb')),None)
        if track and (track.resolved or (track.lost and stamp-track.stamp>=3.)):
            return True
        if not other:
            return False
        if other.get('nbv_rgb') is not None and other.get('nbv_rgb')==self.current.get('nbv_rgb'):
            return False
        if wall_due and other.get('r32_kind')=='interior_surface':
            return True
        return other.get('nbv_score',0.)>self.current.get('nbv_score',.18)+HYSTERESIS

    def dispatched(self,owner,goal):
        super().dispatched(owner,goal)
        self.read_targets(owner)
        self.confirm_pending=False
        self.memory.dispatched(goal,owner.get_clock().now().nanoseconds/1e9)
        self.changed_at=owner.now()
        self.checked_revision=self.memory.revision
        self.current=goal
        record={k:v for k,v in goal.items() if k.startswith('nbv_') or k in
                ('confidence','uv','observations','last_seen','range_m','cue_quality','measured_odom')}
        record.update(elapsed=owner.now()-owner.started,
                      track_count=len(self.memory.tracks),event=self.memory.event,
                      point_map=goal.get('print_point'),stationary=goal.get('stationary',False),
                      rgb_evidence=self.memory.telemetry.copy(),exposures=self.schedule.exposures,
                      action=goal.get('r32_kind','flow_or_marker'),
                      quality={str(k):round(self.schedule.faces[k].best,3) for k in goal.get('nbv_gain',{})})
        try:
            with (Path(owner.p('output_dir'))/'marker_nbv.jsonl').open('a') as log:
                log.write(json.dumps(record,allow_nan=False)+'\n')
        except (OSError, ValueError) as exc:
            owner.get_logger().warning(str(exc))
