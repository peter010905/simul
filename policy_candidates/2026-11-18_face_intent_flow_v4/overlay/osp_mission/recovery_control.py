"""Mission-owned recovery for navigation failures and observation turns."""
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import BackUp, FollowPath

from osp_mission.local_recovery import LocalRecovery
from osp_mission.navigation_flow import NavigationFlow


def navigation_tree():
    return str(Path(get_package_share_directory('nav2_bt_navigator')) /
               'behavior_trees' / 'navigate_w_replanning_time.xml')


class RecoveryFlow(NavigationFlow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.local = LocalRecovery()
        self.pool = {}
        self.shared_exit = False
        self.begin_navigation()

    def begin_navigation(self):
        super().begin_navigation()
        self.best_heading_error = math.inf
        self.last_heading_gain = -math.inf

    def observe_heading(self, yaw, now):
        # Use the departure near the start. Once moving, translation owns progress.
        if self.current_goal is None or self.local.position is None:
            return
        if self.selected_at is None or math.dist(self.selected_at, self.local.position) > .20:
            return
        bearing = self.local.bearing(self.departure, now)
        if bearing is None:
            return
        error = abs(math.remainder(bearing - yaw, 2 * math.pi))
        if error + math.radians(12) < self.best_heading_error:
            if math.isfinite(self.best_heading_error):
                self.last_heading_gain = now
            self.best_heading_error = error

    def stalled(self, now):
        if not super().stalled(now):
            return False
        aligning = now - self.last_heading_gain <= 4. and now - self.progress_time < 20.
        return not aligning

    def departure_point(self, goal, transform):
        if 'departure' in goal:
            x, y = goal['departure']
            return self.goal_point(dict(x=x, y=y), transform)
        return self.goal_point(goal, transform)

    def selected(self, goal, transform):
        super().selected(goal, transform)
        self.departure = self.departure_point(goal, transform)
        self.selected_at = self.local.position

    def failure_direction(self):
        if (self.local.position is not None and getattr(self, 'selected_at', None) is not None
                and math.dist(self.local.position, self.selected_at) < .15):
            return self.departure
        return self.current_goal

    def filter_goals(self, goals, transform, now):
        # Failed destinations keep their cooldown. A shared departure ranks below
        # a fresh direction, but cannot erase every untried policy destination.
        destination_blocked = super().blocked
        eligible = [g for g in goals if not destination_blocked(g, transform, now)]
        preferred = [g for g in eligible if not self.direction_blocked(g, transform, now)]
        self.shared_exit = bool(eligible) and not preferred
        kept = preferred or eligible
        self.filtered_goals += len(goals) - len(kept)
        self.pool = dict(proposed=len(goals), eligible=len(eligible), selected_from=len(kept),
                         shared_exit=self.shared_exit)
        return kept

    def direction_blocked(self, goal, transform, now):
        return not goal.get('stationary') and self.local.excludes(self.departure_point(goal, transform), now)

    def blocked(self, goal, transform, now):
        return (super().blocked(goal, transform, now) or
                (not self.shared_exit and self.direction_blocked(goal, transform, now)))

    def report(self):
        return dict(super().report(), local_recovery=self.local.report(), candidates=self.pool)


def navigation_finished(mission, result):
    flow = getattr(mission, 'navigation_flow', None)
    if flow is None:
        return
    now = mission.now()
    reason = getattr(mission, 'cancel_reason', '')
    flow.record_navigation_result(result.status, reason, now)
    local = getattr(flow, 'local', None)
    if local is None:
        return
    code = getattr(getattr(result, 'result', None), 'error_code', None)
    if reason == 'navigation_stalled' and result.status != 4:
        failure = reason
    elif result.status == 6 and not reason and code in (
        FollowPath.Result.PATIENCE_EXCEEDED, FollowPath.Result.FAILED_TO_MAKE_PROGRESS,
        FollowPath.Result.NO_VALID_CONTROL,
    ):
        failure = f'controller_error_{code}'
    else:
        return
    if local.failed(flow.failure_direction(), failure, now) and local.request_retreat(now):
        mission.recovery_pending = 'local_blockage'
        mission.retreat_deadline = now + 1.5
    mission.get_logger().warning(
        f'Local navigation failure: {failure}; failures near this pose: {local.failures}')


def try_retreat(mission, flow):
    """Dispatch at most once at a stuck pose, through the existing safety checks."""
    local_request = mission.recovery_pending == 'local_blockage'
    mission.recovery_pending = False
    now = mission.now()
    local = getattr(flow, 'local', None)
    if local_request and (local is None or local.anchor is None or local.retreat_used
                          or now > mission.retreat_deadline or now - local.position_time > 1.5):
        return False
    scan_age = (mission.get_clock().now().nanoseconds / 1e9 - mission.scan_stamp
                if mission.scan_stamp is not None else math.inf)
    rear_ready = mission.rear_clear and -.1 <= scan_age <= .5
    if (not rear_ready or not mission.backup.server_is_ready()
            or now - mission.last_recovery < 15.
            or (flow is not None and not flow.backup_allowed(now))):
        if (local_request and now < mission.retreat_deadline and not rear_ready
                and now - mission.last_recovery >= 15. and flow.backup_allowed(now)):
            mission.recovery_pending = 'local_blockage'
            mission.zero()
            return True
        return False
    if local_request:
        local.retreat_used = True
    mission.last_recovery = now
    goal = BackUp.Goal()
    goal.target.x = .20
    goal.speed = .08
    goal.time_allowance.sec = 8
    mission.send(mission.backup, goal, 'backup')
    return True
