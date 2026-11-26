"""Budget a late full-camera sweep while the original mission owns every action."""
import json
import math
from pathlib import Path
import numpy as np
from osp_mission.box_views import odom_point


def rotation_seconds(angle, speed, acceleration):
    if speed<=0 or acceleration<=0:
        raise ValueError('Rotation speed and acceleration must be positive')
    angle=abs(angle)
    ramp_angle=speed*speed/acceleration
    if angle<=ramp_angle:
        return 2*math.sqrt(angle/acceleration)
    return angle/speed+speed/acceleration


class Rescue360:
    def __init__(self,owner):
        self.owner=owner
        values=[owner.p(k) for k in ('rescue_after_s','rescue_travel_speed','rescue_settle_s')]
        if not np.isfinite(values).all() or values[0]<0 or values[1]<=0 or values[2]<0:
            raise ValueError('Invalid timed exploration configuration')
        self.active=False
        self.target=None
        self.spin_pending=False
        self.spin_running=False
        self.rotation_done=0.
        self.spin_start_frames=0
        self.visited=[]
        self.failed=[]
        self.settle_until=0.
        self.awaiting_frames=None
        self.completed_sweeps=0
        self.last_empty_reason=''
        self.rotation_s=rotation_seconds(2*math.pi,owner.p('rescue_spin_rate'),owner.p('rescue_spin_accel'))

    def elapsed(self):
        return self.owner.now()-self.owner.started

    def due(self):
        return self.elapsed()>=self.owner.p('rescue_after_s')

    def transition(self):
        owner=self.owner
        if self.active or not self.due():
            return False
        busy=bool(owner.goal_handle or owner.pending)
        protected=(getattr(owner,'batch_approach_id',None) is not None
                   or bool(getattr(getattr(owner,'marker_look',None),'pending',None))
                   or bool(getattr(owner,'registered_pending',set()))
                   or bool(getattr(owner,'readable_cue',None)))
        guard=getattr(owner,'contact_inspection',None)
        if protected or (guard and guard.phase) or getattr(owner,'recovery_pending',False):
            return False
        if busy and getattr(owner,'kind','')!='nav':
            return False
        self.active=True
        self.write('activate')
        if busy:
            owner.cancel_action('rescue_phase_due')
            owner.zero()
            return True
        owner.scan_after=False
        owner.plan_after=owner.now()
        return False

    def waiting(self):
        if self.owner.now()<self.settle_until:
            return True
        if self.awaiting_frames is None:
            return False
        count=self.owner.camera_diagnostics['coverage_updates']
        if count-self.awaiting_frames>=3:
            self.awaiting_frames=None
            self.write('fresh_frames_after_sweep')
            return False
        # Sensor freshness still uses the original readiness guard. Bound this
        # extra dwell if the detector stays alive but receives no new exposure.
        if self.owner.now()-self.settle_until<2.:
            return True
        self.awaiting_frames=None
        self.write('post_sweep_frame_wait_expired')
        return False

    def choose(self,choices,pose,transform,grid,cost,nav,veto,weights):
        owner=self.owner
        now=owner.now()
        elapsed=self.elapsed()
        remaining=owner.p('duration')-elapsed
        overhead=self.rotation_s+owner.p('rescue_settle_s')+3.
        self.failed=[(p,t) for p,t in self.failed if now-t<45.]
        ranked=[]
        for goal in choices:
            if goal.get('stationary') or veto(goal):
                continue
            cell=cost.index(goal['x'],goal['y'])
            if not cost.inside(*cell) or not nav[cell]:
                continue
            surface,frontier,distance,_,_=goal['features']
            if not np.isfinite([surface,frontier,distance]).all():
                continue
            gain=float(weights[0]*surface+weights[1]*frontier)
            if gain<=0 or distance<=0:
                continue
            planned_s=float(distance)/owner.p('rescue_travel_speed')+overhead
            if planned_s>remaining:
                continue
            odom=odom_point((goal['x'],goal['y']),transform)
            if any(np.linalg.norm(odom-p)<1.25 for p in self.visited):
                continue
            if any(np.linalg.norm(odom-p)<1.25 for p,_ in self.failed):
                continue
            # Full-circle visual gain is measured by the existing safe sampler.
            # Prefer the greatest benefit, with travel as the tie-breaker.
            chosen=dict(goal,scan_angle=2*math.pi,rescue360=True,r32_kind='rescue360',
                        rescue_odom=odom.tolist(),rescue_expected_s=planned_s,
                        rescue_surface_gain=float(surface),rescue_frontier_gain=float(frontier),
                        rescue_information_gain=gain)
            ranked.append((gain,-float(distance),chosen))
        if not ranked:
            self.last_empty_reason='no_budgeted_safe_information_view'
            return None
        self.last_empty_reason=''
        return max(ranked,key=lambda item:item[:2])[2]

    def dispatched(self,goal):
        if not goal.get('rescue360'):
            self.target=None
            self.spin_pending=False
            return
        self.target=goal
        self.spin_pending=False
        self.write('goal_selected',dict(goal=[goal['x'],goal['y'],goal['yaw']],
            surface_gain=goal['rescue_surface_gain'],frontier_gain=goal['rescue_frontier_gain'],
            estimated_total_s=goal['rescue_expected_s']))

    def spin_sent(self):
        self.spin_running=True
        self.rotation_done=0.
        self.spin_start_frames=self.owner.camera_diagnostics['coverage_updates']
        self.write('full_sweep_sent',dict(target_angle_rad=2*math.pi,rotation_estimate_s=self.rotation_s))

    def outcome(self,kind,status,reason,progress):
        owner=self.owner
        sensor_reasons={'processed_camera_stale','scan_stale','map_unavailable',
                        'map_to_base_tf_unavailable','nav2_unavailable','drive_unavailable'}
        if status==5 and reason in sensor_reasons:
            if kind=='spin' and self.spin_running:
                self.rotation_done+=abs(progress)
            self.write('paused_for_sensor_recovery',dict(kind=kind,reason=reason))
            return
        if kind=='nav' and self.target is not None:
            if status==4:
                self.spin_pending=True
                self.write('arrived')
            else:
                if reason not in ('registered_marker','readable_marker','marker_hint'):
                    self.failed.append((np.array(self.target['rescue_odom']),owner.now()))
                self.write('navigation_ended',dict(status=status,reason=reason))
                self.target=None
                self.spin_pending=False
        elif kind=='spin' and self.spin_running:
            progress=self.rotation_done+abs(progress)
            camera_frames=owner.camera_diagnostics['coverage_updates']-self.spin_start_frames
            complete=status==4 and progress>=2*math.pi-.15 and camera_frames>=3
            if complete and self.target is not None:
                self.visited.append(np.array(self.target['rescue_odom']))
                self.completed_sweeps+=1
                self.settle_until=owner.now()+owner.p('rescue_settle_s')
                self.awaiting_frames=owner.camera_diagnostics['coverage_updates']
            elif self.target is not None:
                self.failed.append((np.array(self.target['rescue_odom']),owner.now()))
            self.write('full_sweep_result',dict(status=status,reason=reason,measured_angle_rad=progress,camera_evidence_frames=camera_frames,completed=complete))
            self.spin_pending=False
            self.spin_running=False
            self.target=None

    def write(self,event,extra=None):
        owner=self.owner
        row=dict(event=event,elapsed=self.elapsed(),completed_sweeps=self.completed_sweeps,
                 remaining_s=owner.p('duration')-self.elapsed(),**(extra or {}))
        try:
            with (Path(owner.p('output_dir'))/'rescue360.jsonl').open('a') as stream:
                stream.write(json.dumps(row,allow_nan=False)+'\n')
        except OSError as exc:
            owner.get_logger().warning(str(exc))
