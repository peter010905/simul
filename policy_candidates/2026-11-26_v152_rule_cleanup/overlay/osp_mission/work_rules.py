"""Shared exploration rules; navigation and RGB evidence retain safety ownership."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class WorkRules:
    normal_clearance_m: float = .32
    progress_speed_m_s: float = .20
    inspection_reserve_s: float = 8.
    box_visit_s: float = 55.
    box_return_cooldown_s: float = 90.
    face_retry_s: float = 45.
    face_retries_per_geometry: int = 2
    face_retries_total: int = 6
    same_face_m: float = .50
    same_box_m: float = .40
    direction_cosine: float = .70
    frontier_gain_min: float = .15
    coverage_marker_min_m: float = .08
    readable_pixels: float = 18.


RULES = WorkRules()


def same_face(a, b):
    return (a.get('surface_odom') is not None and b.get('surface_odom') is not None
            and math.dist(a['surface_odom'], b['surface_odom']) < RULES.same_face_m
            and math.cos(a['normal_odom']-b['normal_odom']) > RULES.direction_cosine)


def marker_task_dispatched(focus, approach, goal, now):
    """Commit only the final selected task, never intermediate proposals."""
    if 'focus_id' in goal:
        focus.dispatched(goal, now)
    if 'approach_id' in goal:
        approach.dispatched(goal, now)


class FaceWork:
    """Bounded failures per physical face, reset only by meaningful local changes.

    OBSERVED is diagnostic and can only be set by the existing RGB coverage
    gates. A navigation or spin result alone never grants camera credit.
    """
    def __init__(self):
        self.rows = []
        self.active = None

    def row(self, point, yaw):
        found = next((r for r in self.rows if math.dist(point, r['point']) < RULES.same_face_m
                      and math.cos(yaw-r['yaw']) > RULES.direction_cosine), None)
        if found is None:
            found = dict(point=tuple(point), yaw=yaw, status='PENDING', failures=[], total=0,
                         geometry=None, reason='', reconsiderations=0)
            self.rows.append(found)
            self.rows = self.rows[-256:]
        return found

    def probe(self, point, yaw, geometry):
        row = self.row(point, yaw)
        old = row['geometry']
        if old is not None and (len(old) != len(geometry) or
                sum(a != b for a,b in zip(old,geometry)) >= max(3, math.ceil(.08*len(geometry)))):
            row['failures'] = []
            row['status'] = 'PENDING'
            row['reconsiderations'] += 1
        row['geometry'] = tuple(geometry)
        return row

    def attempts(self, point, yaw):
        return self.row(point, yaw)['failures']

    def allowed(self, point, yaw, elapsed):
        row = self.row(point, yaw)
        allowed = (row['total'] < RULES.face_retries_total
                and len(row['failures']) < RULES.face_retries_per_geometry
                and (not row['failures'] or elapsed-row['failures'][-1] >= RULES.face_retry_s))
        if not allowed:
            row['status']='FAILED'
            row['reason']='retry_budget' if row['total'] >= RULES.face_retries_total or len(row['failures']) >= RULES.face_retries_per_geometry else 'retry_cooldown'
        return allowed

    def dispatched(self, goal):
        self.active = dict(goal) if goal.get('face_inspection') and 'surface_odom' in goal else None

    def result(self, kind, status, reason, elapsed):
        if self.active is None or kind not in ('nav','spin'):
            return
        row = self.row(self.active['surface_odom'], self.active['normal_odom'])
        if status == 4:
            if kind == 'spin':
                self.active = None
            return
        from osp_mission.action_priority import SENSOR_REASONS
        if reason in SENSOR_REASONS or reason in ('marker_hint','readable_marker','registered_marker','inspection_due'):
            return
        row['status'], row['reason'] = 'FAILED', reason or 'action_failed'
        row['failures'].append(elapsed)
        row['total'] += 1
        self.active = None

    def summary(self):
        return [dict(point=r['point'], yaw=r['yaw'], status=r['status'], reason=r['reason'],
                     failures=len(r['failures']), total_failures=r['total'],
                     reconsiderations=r['reconsiderations']) for r in self.rows]
