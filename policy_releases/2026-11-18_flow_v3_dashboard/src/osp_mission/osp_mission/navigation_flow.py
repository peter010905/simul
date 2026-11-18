"""Reduce optional stops while preserving marker and safety interruptions."""
import math


class NavigationFlow:
    def __init__(self, allow_mid_route_wall_inspection=False):
        self.allow_mid_route_wall_inspection = allow_mid_route_wall_inspection
        self.settle = None
        self.current_goal = None
        self.failures = []
        self.backup_blocked_until = -math.inf
        self.progress_position = None
        self.progress_time = None
        self.skipped_zero_turns = 0
        self.stalls_cancelled = 0
        self.filtered_goals = 0

    def begin_settle(self, now, frames):
        self.settle = (float(now), int(frames))

    def waiting_for_frames(self, now, frames):
        if self.settle is None:
            return False
        started, initial = self.settle
        elapsed = now-started
        if elapsed < .5 or (frames-initial < 3 and elapsed < 1.5):
            return True
        self.settle = None
        return False

    @staticmethod
    def goal_point(goal, transform):
        x, y = float(goal['x']), float(goal['y'])
        if transform is None:
            return ('map', x, y)
        tx, ty, angle = transform
        c, s = math.cos(angle), math.sin(angle)
        return ('odom', tx+c*x-s*y, ty+s*x+c*y)

    def selected(self, goal, transform):
        self.current_goal = self.goal_point(goal, transform)

    def record_navigation_result(self, status, reason, now):
        resumable = {'processed_camera_stale','scan_stale','map_unavailable',
            'map_to_base_tf_unavailable','nav2_unavailable','drive_unavailable',
            'marker_hint','near_pass_inspection','near_pass_time_limit'}
        if status == 4 or (status == 5 and reason in resumable):
            return
        if self.current_goal is not None:
            self.failures.append((self.current_goal, float(now)+60.))
            self.failures = self.failures[-64:]

    def blocked(self, goal, transform, now):
        self.failures = [row for row in self.failures if now < row[1]]
        point = self.goal_point(goal, transform)
        return any(point[0] == old[0] and math.hypot(point[1]-old[1], point[2]-old[2]) < .45
                   for old, _ in self.failures)

    def filter_goals(self, goals, transform, now):
        kept = [goal for goal in goals if not self.blocked(goal, transform, now)]
        self.filtered_goals += len(goals)-len(kept)
        return kept

    def begin_navigation(self):
        self.progress_position = None
        self.progress_time = None

    def observe_position(self, x, y, now):
        if not all(math.isfinite(v) for v in (x, y, now)):
            return
        if self.progress_position is None or math.hypot(x-self.progress_position[0], y-self.progress_position[1]) >= .06:
            self.progress_position = (x, y)
            self.progress_time = now

    def stalled(self, now):
        return self.progress_time is not None and now-self.progress_time >= 8.

    def backup_result(self, status, now):
        if status != 4:
            self.backup_blocked_until = now+60.

    def backup_allowed(self, now):
        return now >= self.backup_blocked_until

    def report(self):
        return dict(mid_route_wall_inspection=self.allow_mid_route_wall_inspection,
            skipped_zero_turn_waits=self.skipped_zero_turns,
            stalls_cancelled=self.stalls_cancelled, failed_goals_filtered=self.filtered_goals,
            failed_goal_cooldown_s=60., stall_timeout_s=8., minimum_progress_m=.06,
            observation_settle_min_s=.5, observation_settle_max_s=1.5,
            observation_fresh_frames=3)
