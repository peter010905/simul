"""Cancel,look,dwell,and resume a bounded exploration navigation safely."""
import copy
import json
import math
from pathlib import Path

import numpy as np
from nav2_msgs.action import Spin
from rclpy.time import Time

from osp_mission.near_pass_inspection import NearPassInspection
from osp_mission.policy import wrap


class NearPassRuntime:
    def __init__(self,owner):
        self.owner=owner;self.policy=NearPassInspection();self.task=None
        self.protected_navigation=False

    def camera_pose(self,frame):
        transform=self.owner.tf.lookup_transform(frame,self.owner.inspection_camera_frame,Time()).transform
        q=transform.rotation
        return (transform.translation.x,transform.translation.y,
            math.atan2(2*(q.y*q.z-q.w*q.x),2*(q.x*q.z+q.w*q.y)))

    def event(self,event,**details):
        owner=self.owner
        try:
            with (Path(owner.p('output_dir'))/'near_pass_inspections.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(event=event,stamp=owner.now(),**details))+'\n')
        except OSError as exc:owner.get_logger().warning(f'Near-pass audit unavailable: {exc}')

    def request(self):
        owner=self.owner
        if (self.task is not None or not owner.edge_mode or owner.kind!='nav'
                or owner.goal_handle is None or owner.pending or owner.cancel_reason
                or self.protected_navigation or owner.now()-owner.action_start<3.
                or owner.now()<owner.spin_blocked_until):return
        calibration=getattr(owner,'inspection_camera',None)
        if owner.map is None or calibration is None:return
        try:
            grid=owner.map;data=np.asarray(grid.data).reshape(grid.info.height,grid.info.width)
            origin=(grid.info.origin.position.x,grid.info.origin.position.y)
            base=owner.tf.lookup_transform(owner.p('map_frame'),owner.p('base_frame'),Time()).transform.translation
            camera=self.camera_pose(owner.p('map_frame'));odom=self.camera_pose('odom')
            viewed=owner.coverage.grid(data,grid.info.resolution,origin)
            elapsed=owner.now()-owner.started
            choice=self.policy.choose(data,grid.info.resolution,origin,camera,(base.x,base.y),
                viewed,*calibration,odom,owner.now(),elapsed,owner.p('duration')-elapsed)
            if choice is None:return
            self.policy.reserve(choice,owner.now())
            self.task=dict(phase='cancel_nav',resume_goal=copy.deepcopy(owner.active_goal),
                desired_odom_yaw=choice['odom_yaw'],started=owner.now(),
                cue_stamps={ident:hint['stamp'] for ident,hint in owner.marker_focus.hints.items()},
                confirmed_ids=set(owner.targets))
            self.event('requested',choice=choice)
            owner.cancel_action('near_pass_inspection');owner.zero()
        except Exception as exc:owner.get_logger().debug(f'Near-pass opportunity unavailable: {exc}')

    def abort(self,reason,retain_navigation=False):
        if self.task is None:return
        owner=self.owner;task=self.task
        self.event('finished',reason=reason,resumed_navigation=retain_navigation)
        self.policy.finish(owner.now(),reason);self.task=None
        if retain_navigation:owner.resume_action=('nav',task['resume_goal'])

    def poll(self):
        if self.task is None:return
        limit=min(10.,max(0.,self.policy.maximum_total_sim_s-self.policy.spent))
        if self.owner.now()-self.task['started']<limit:return
        owner=self.owner
        if not owner.cancel_reason:owner.cancel_action('near_pass_time_limit')
        if owner.goal_handle is not None:owner.goal_handle.cancel_goal_async()
        self.abort('inspection_time_limit')
        owner.scan_after=False

    def completed(self,status,kind,cancel_reason):
        if self.task is None:return False
        owner=self.owner;phase=self.task['phase']
        sensors={'processed_camera_stale','scan_stale','map_unavailable','map_to_base_tf_unavailable','nav2_unavailable','drive_unavailable'}
        if status==5 and cancel_reason in sensors:
            self.abort('sensor_interruption',retain_navigation=True)
            owner.sensor_interruptions+=1;owner.scan_after=False
            return True
        if phase=='cancel_nav' and kind=='nav':
            if status!=5 or cancel_reason!='near_pass_inspection':
                self.abort('navigation_finished_before_inspection');return False
            self.task['phase']='ready_turn';owner.scan_after=False
            return True
        if phase=='turning' and kind=='spin':
            if status!=4:
                self.abort('spin_not_successful');owner.scan_after=False
                owner.spin_blocked_until=owner.now()+30.;owner.recovery_pending=True
                return True
            self.task.update(phase='dwell',frames_before=owner.camera_diagnostics['accepted'],
                dwell_started=owner.now())
            owner.scan_after=False
            return True
        return False

    def rejected(self,kind):
        if self.task is None or kind!='spin':return False
        owner=self.owner;self.abort('spin_rejected');owner.scan_after=False
        owner.spin_blocked_until=owner.now()+30.;owner.recovery_pending=True
        return True

    def tick(self):
        if self.task is None:return False
        owner=self.owner;task=self.task
        if task['phase']=='ready_turn':
            try:turn=wrap(task['desired_odom_yaw']-self.camera_pose('odom')[2])
            except Exception:return True  # Fresh TF is required;poll bounds the wait.
            if abs(turn)>self.policy.maximum_turn_rad+.05:
                self.abort('view_moved_outside_angular_cap',retain_navigation=True);return True
            goal=Spin.Goal();goal.target_yaw=turn;goal.time_allowance.sec=8
            task['phase']='turning';self.event('turn_started',turn_rad=turn)
            owner.send(owner.spin,goal,'spin');return True
        if task['phase']=='dwell':
            owner.zero();elapsed=owner.now()-task['dwell_started']
            frames=owner.camera_diagnostics['accepted']-task['frames_before']
            if elapsed<.8 or (frames<3 and elapsed<2.):return True
            new_evidence=(any(ident not in task['confirmed_ids'] and
                hint['stamp']>task['cue_stamps'].get(ident,-math.inf)
                for ident,hint in owner.marker_focus.hints.items())
                or bool(set(owner.targets)-task['confirmed_ids']))
            self.abort('new_marker_evidence' if new_evidence else 'processed_frame_dwell',
                retain_navigation=not new_evidence)
            # New marker evidence returns to the normal focus/approach planner.
            owner.scan_after=False;owner.plan_after=owner.now()
            return True
        return True
