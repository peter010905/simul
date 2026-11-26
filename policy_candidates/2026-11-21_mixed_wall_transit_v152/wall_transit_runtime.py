"""Reuse acknowledged Nav2 cancel/Spin/frame-dwell/resume, with bounded timeout."""
from osp_mission.near_pass_runtime import NearPassRuntime
from osp_mission.wall_transit_inspection import WallTransitInspection


class WallTransitRuntime(NearPassRuntime):
    def __init__(self, owner):
        super().__init__(owner)
        self.policy = WallTransitInspection()
        self.next_probe = 0.

    def request(self):
        owner = self.owner
        if owner.now() < self.next_probe:
            return
        self.next_probe = owner.now()+2.
        guard = getattr(owner, 'contact_inspection', None)
        marker = getattr(getattr(owner, 'marker_look', None), 'pending', None) or {}
        if (getattr(owner, 'state', '') != 'running'
                or getattr(owner, 'contact_pending', False) or getattr(owner, 'recovery_pending', False)
                or (guard is not None and (guard.phase or (guard.bearing and owner.now()-guard.bearing[0] <= 2.)))
                or marker.get('kind') in ('approach', 'focus', 'anonymous_print')):
            return
        super().request()

    def new_evidence(self):
        task, owner = self.task, self.owner
        return task is not None and (bool(set(owner.targets)-task['confirmed_ids']) or
            any(i not in task['confirmed_ids'] and hint['stamp'] > task['cue_stamps'].get(i, -float('inf'))
                for i, hint in owner.marker_focus.hints.items()))

    def release_for_marker(self):
        owner = self.owner
        self.abort('new_marker_evidence')
        for name in ('marker_look', 'face_look'):
            look = getattr(owner, name, None)
            if look is not None:
                look.clear('new_marker_evidence')
        owner.scan_after = False
        owner.plan_after = owner.now()

    def poll(self):
        if self.task is None:
            return
        owner = self.owner
        if not (owner.goal_handle or owner.pending) and self.task['phase'] in ('cancel_nav', 'turning'):
            # A failed action future is not proof that the motor action ended.
            self.abort('action_state_unknown')
            owner.zero()
            owner.finish('wall_transit_action_state_unknown')
            return
        limit = min(10., max(0., self.policy.maximum_total_sim_s-self.policy.spent))
        if owner.now()-self.task['started'] < limit:
            return
        # Keep the task until cancellation is acknowledged; never resume over a live Spin.
        if not (owner.goal_handle or owner.pending):
            self.abort('inspection_time_limit', retain_navigation=True)
            owner.scan_after = False
            return
        self.task['timed_out'] = True
        owner.cancel_action('near_pass_time_limit')
        owner.zero()

    def completed(self, status, kind, cancel_reason):
        if self.task is not None:
            if status == 5 and cancel_reason in {'processed_camera_stale', 'scan_stale',
                    'map_unavailable', 'map_to_base_tf_unavailable', 'nav2_unavailable', 'drive_unavailable'}:
                return super().completed(status, kind, cancel_reason)
            if self.new_evidence() and status in (4, 5):
                self.release_for_marker()
                return True
            if self.task.get('timed_out') and status == 5 and cancel_reason == 'near_pass_time_limit':
                self.abort('inspection_time_limit', retain_navigation=True)
                self.owner.scan_after = False
                return True
        return super().completed(status, kind, cancel_reason)

    def tick(self):
        if self.task is not None and self.new_evidence():
            owner = self.owner
            if owner.goal_handle or owner.pending:
                owner.cancel_action('registered_marker' if set(owner.targets)-self.task['confirmed_ids'] else 'marker_hint')
                owner.zero()
            else:
                self.release_for_marker()
            return True
        if self.task is not None and self.task.get('timed_out'):
            self.owner.zero()
            return True
        if self.task is not None and self.task['phase'] == 'dwell':
            owner, task = self.owner, self.task
            elapsed = owner.now()-task['dwell_started']
            frames = owner.camera_diagnostics['accepted']-task['frames_before']
            if elapsed >= .8 and (frames >= 3 or elapsed >= 2.):
                self.event('wall_view_frames', processed_frames=frames,
                    dwell_s=elapsed, minimum_met=frames >= 3)
                if frames < 3:
                    self.abort('wall_view_no_fresh_frames', retain_navigation=True)
                    owner.scan_after = False
                    return True
        return super().tick()
