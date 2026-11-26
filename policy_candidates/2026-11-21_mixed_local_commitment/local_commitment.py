"""At most one safe local surface before a distant, unconfirmed marker focus."""
from collections import deque
import json
import math
from pathlib import Path
import numpy as np
from rclpy.time import Time
from osp_mission.marker_focus import MarkerFocus
from osp_mission.phased_hybrid import PhasedHybrid
from osp_mission.box_views import odom_point
from osp_mission.wall_route import wrap


class CommitmentBudget:
    """No permanent suppression: one extra face per box in a 120s window."""
    def __init__(self):
        self.used=deque(maxlen=32)

    def allowed(self,box,now):
        self.used=deque((row for row in self.used if 0<=now-row[1]<120),maxlen=32)
        return not any(math.dist(box,old)<.4 for old,_ in self.used)

    def dispatched(self,box,now):
        self.used.append((tuple(box),float(now)))


def worth_one_face(local,focus,robot,heading,hint_point,box,remaining):
    if not local or local.get('r32_kind')!='interior_surface' or not local.get('local_batch'):
        return False
    if local.get('previous_attempts') or local.get('completed_box') or box is None:
        return False
    # Close evidence and evidence on the carrier itself remain immediate.
    if math.dist(robot,hint_point)<=1.5 or math.dist(box,hint_point)<=1.25:
        return False
    distance=float(local.get('distance',math.dist(robot,(local['x'],local['y']))))
    seconds=distance/.25+abs(wrap(local['yaw']-heading))/.5+abs(local.get('scan_angle',0.))/.5+.5
    focus_seconds=float(focus.get('distance',0.))/.25+8.
    return seconds<=20. and remaining>=seconds+focus_seconds+10.


def transform(owner):
    t=owner.tf.lookup_transform('odom',owner.p('map_frame'),Time()).transform
    q=t.rotation
    return t.translation.x,t.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class CommittedFocus(MarkerFocus):
    def __init__(self,owner):
        super().__init__();self.owner=owner

    def request_interrupt(self,ident,confirmed,now):
        hybrid=getattr(self.owner,'hybrid',None)
        if hybrid is not None and hybrid.finish_nearby_transit(self.owner,self.hints.get(ident),now):
            return False
        return super().request_interrupt(ident,confirmed,now)

    def choose(self,data,resolution,origin,robot,heading,candidates,confirmed,now):
        attempts=dict(self.attempts);times=dict(self.last_attempt)
        focus=super().choose(data,resolution,origin,robot,heading,candidates,confirmed,now)
        # choose() is a proposal. Another confirmed approach may win later in
        # Mission.tick; only dispatched() may consume a focus attempt/cooldown.
        self.attempts=attempts;self.last_attempt=times
        if focus is None:return None
        focus=dict(focus,proposed_attempt=attempts.get(focus['focus_id'],0)+1)
        hybrid=getattr(self.owner,'hybrid',None)
        local=hybrid.local_alternative(self.owner,focus,self.hints[focus['focus_id']],data,
                                        robot,heading,candidates,now) if hybrid is not None else None
        if local is None:return focus
        # A proposal that was not dispatched must not consume its focus budget.
        hybrid.focus_alternative=local
        return None


class CommittedHybrid(PhasedHybrid):
    def __init__(self):
        super().__init__();self.budget=CommitmentBudget();self.focus_alternative=None
        self.active_surface=None;self.deferrals=0

    def poll_budget(self,owner):
        active=self.active_surface
        if not active or not active['goal'].get('local_focus_commitment'):
            return
        if owner.now()-active['started']<=20. or owner.state!='running':return
        if owner.contact_pending or owner.recovery_pending or owner.contact_inspection.phase:
            return
        if (owner.goal_handle or owner.pending) and owner.kind in ('nav','spin') and not owner.cancel_reason:
            # Reuse the existing intentional hint-yield path. It does not add
            # a navigation failure or mark this unfinished face as visited.
            owner.get_logger().info('Local face budget expired; yielding to remembered marker focus')
            owner.cancel_action('marker_hint');owner.zero()
        self.active_surface=None

    def finish_nearby_transit(self,owner,hint,now):
        active=self.active_surface
        if not active or hint is None or owner.now()-active['started']>20.:
            return False
        if owner.contact_pending or owner.recovery_pending or owner.contact_inspection.phase:
            return False
        box=active['goal'].get('batch_box')
        if box is None or active['goal'].get('completed_box'):
            return False
        try:
            tr=transform(owner)
            pose=owner.tf.lookup_transform(owner.p('map_frame'),owner.p('base_frame'),Time()).transform.translation
            robot=np.array([pose.x,pose.y]);point=np.array(hint['point'])
            c,s=math.cos(tr[2]),math.sin(tr[2]);dx,dy=box[0]-tr[0],box[1]-tr[1]
            box_map=np.array([c*dx+s*dy,-s*dx+c*dy])
            return (math.dist(robot,owner.destination)<=.8 and math.dist(robot,point)>1.5
                    and math.dist(box_map,point)>1.25)
        except Exception:
            return False

    def local_alternative(self,owner,focus,hint,data,robot,heading,choices,now):
        if owner.contact_pending or owner.recovery_pending or owner.contact_inspection.phase:
            return None
        box=self.schedule.batch.active
        if box is None or not self.budget.allowed(box,owner.now()):return None
        try:
            tr=transform(owner)
            from osp_mission.r32_drive import grid_from_message
            mapped=grid_from_message(owner.map)
            viewed=owner.coverage.grid(data,mapped.resolution,mapped.origin[:2])
            local=super().select(owner,None,data,viewed,(*robot,heading),tr,
                print_only=True,box_probe=True,choices=choices)
            if local is None or local.get('batch_box') is None:return None
            if math.dist(box,local['batch_box'])>.4:return None
            hint_odom=odom_point(hint['point'],tr);robot_odom=odom_point(robot,tr)
            goal_odom=odom_point((local['x'],local['y']),tr)
            check=dict(local,x=goal_odom[0],y=goal_odom[1],yaw=local['yaw']+tr[2])
            remaining=owner.p('duration')-(owner.now()-owner.started)
            if not worth_one_face(check,focus,robot_odom,heading+tr[2],hint_odom,box,remaining):
                return None
            return dict(local,local_focus_commitment=True,deferred_focus_id=focus['focus_id'])
        except Exception as exc:
            owner.get_logger().debug(f'Local focus alternative unavailable: {exc}')
            return None

    def select(self,owner,selected,*args,**kwargs):
        local,self.focus_alternative=self.focus_alternative,None
        if selected and any(k in selected for k in ('approach_id','focus_id')):
            return selected
        if local is not None:return local
        return super().select(owner,selected,*args,**kwargs)

    def dispatched(self,owner,goal):
        super().dispatched(owner,goal)
        if 'focus_id' in goal:
            ident=goal['focus_id'];focus=owner.marker_focus
            focus.attempts[ident]=focus.attempts.get(ident,0)+1
            focus.last_attempt[ident]=owner.get_clock().now().nanoseconds/1e9
            try:
                with (Path(owner.p('output_dir'))/'marker_focus_dispatches.jsonl').open('a') as stream:
                    stream.write(json.dumps(dict(time=owner.now(),id=ident,attempt=focus.attempts[ident],
                        goal=[goal['x'],goal['y'],goal['yaw']]))+'\n')
            except OSError as exc:owner.get_logger().warning(str(exc))
        self.active_surface=(dict(goal=dict(goal),started=owner.now())
            if goal.get('r32_kind')=='interior_surface' and not goal.get('previous_attempts') else None)
        if goal.get('local_focus_commitment'):
            self.budget.dispatched(goal['batch_box'],owner.now());self.deferrals+=1
            owner.get_logger().info(f"Finishing one local face before distant ID {goal['deferred_focus_id']}")
            try:
                with (Path(owner.p('output_dir'))/'local_commitments.jsonl').open('a') as stream:
                    stream.write(json.dumps(dict(time=owner.now(),deferred_id=goal['deferred_focus_id'],
                        box=list(goal['batch_box']),goal=[goal['x'],goal['y'],goal['yaw']],maximum_s=20.))+'\n')
            except OSError as exc:owner.get_logger().warning(str(exc))
