"""Keep interrupted surface work without reserving unsafe endpoints or unbounded time."""
import math

from osp_mission.box_tour import short_tour
from osp_mission.work_rules import RULES, same_face
from osp_mission.action_priority import SENSOR_REASONS



def same_view(a, b, radius=.35):
    if not all(k in a and k in b for k in ('x', 'y')):
        return True
    return math.dist((a['x'], a['y']), (b['x'], b['y'])) < radius


def same_side(a, b):
    """.63: same tracked box and same face direction, from any standoff."""
    return (a.get('batch_box') is not None and b.get('batch_box') is not None
            and math.dist(a['batch_box'], b['batch_box']) < RULES.same_box_m
            and math.cos(a['normal_odom']-b['normal_odom']) > RULES.direction_cosine)


class BoxContinuity:
    def __init__(self):
        self.active = None
        self.sweeping = False
        self.suspended = []
        self.box = None
        self.entered = -math.inf
        self.started = -math.inf
        self.events = []
        self.swept = []     # faces already swept from their standoff: (goal, elapsed)
        self.sides = []     # .63: every swept box side, kept for the whole mission

    def event(self, event, elapsed, **fields):
        self.events.append(dict(event=event, elapsed=elapsed, **fields))

    def protecting(self, elapsed):
        return self.active is not None and 0 <= elapsed-self.started <= 65.

    def dispatched(self, goal, elapsed):
        if goal.get('r32_kind') != 'interior_surface' or not goal.get('face_inspection'):
            self.active = None
            self.sweeping = False
            return
        self.suspended = [(g,t) for g,t in self.suspended if not same_face(g,goal)]
        self.active = dict(goal)
        self.started = elapsed
        self.sweeping = False
        center = goal.get('batch_box')
        if center is not None and (self.box is None or math.dist(center, self.box) >= RULES.same_box_m
                                   or elapsed-self.entered >= RULES.box_visit_s+RULES.box_return_cooldown_s):
            self.box, self.entered = tuple(center), elapsed
        self.event('face_dispatched', elapsed, surface=goal['surface_odom'],
                   resumed=bool(goal.get('box_face_resume')))

    def begin_sweep(self, turn, elapsed):
        if self.active is not None:
            self.sweeping = True
            if abs(turn) < .04:
                self.result('spin', 4, '', elapsed)

    def result(self, kind, status, reason, elapsed):
        if self.active is None or kind not in ('nav', 'spin'):
            return None
        if status == 4 and (kind == 'nav' or not self.sweeping):
            return None
        if status == 5 and reason in SENSOR_REASONS:
            return None
        goal, self.active = self.active, None
        self.sweeping = False
        if status == 4:
            self.event('sweep_finished', elapsed, surface=goal['surface_odom'])
            self.swept = [(g,t) for g,t in self.swept if not same_face(g,goal)][-11:]+[(goal,elapsed)]
            if goal.get('batch_box') is not None and not self.side_swept(goal):
                self.sides = self.sides[-47:]+[goal]
            return None
        if reason not in ('marker_hint', 'readable_marker', 'registered_marker', 'inspection_due'):
            self.event('face_failed', elapsed, reason=reason, surface=goal['surface_odom'])
            return None
        if goal.get('face_resume_count', 0) >= 2:
            self.event('resume_limit', elapsed, surface=goal['surface_odom'])
            return None
        if goal.get('face_resume_count', 0) < 2:
            self.suspended = [(g,t) for g,t in self.suspended if not same_face(g,goal)]
            self.suspended.append((goal,elapsed))
            self.suspended = self.suspended[-12:]
        self.event('face_suspended', elapsed, reason=reason, surface=goal['surface_odom'])
        return goal

    def available(self, goal, elapsed):
        center = goal.get('batch_box')
        if center is None or self.box is None or math.dist(center,self.box) >= RULES.same_box_m:
            return True
        age = elapsed-self.entered
        return age < RULES.box_visit_s or age >= RULES.box_visit_s+RULES.box_return_cooldown_s

    def recently_swept(self, goal, elapsed):
        # Sweeping a face again from the same standoff yields nothing new; another view may.
        hold = RULES.box_visit_s+RULES.box_return_cooldown_s
        return any(same_face(g,goal) and 0 <= elapsed-t < hold and same_view(g,goal)
                   for g,t in self.swept)

    def side_swept(self, goal):
        return any(same_side(g, goal) for g in self.sides)

    def unswept_first(self, ranked):
        """Finish a box: while it still offers a face direction never swept, drop its swept ones."""
        fresh = [g['batch_box'] for _, g in ranked
                 if g.get('batch_box') is not None and not self.side_swept(g)]
        return [(walk, g) for walk, g in ranked
                if not (self.side_swept(g)
                        and any(math.dist(g['batch_box'], box) < RULES.same_box_m for box in fresh))]

    def choose(self, ranked, elapsed, remaining, grid, safe):
        ranked = [(walk,g) for walk,g in ranked if self.available(g,elapsed)]
        self.suspended = [(g,t) for g,t in self.suspended if 0 <= elapsed-t <= 90.]
        for previous, interrupted_at in self.suspended:
            options = [(walk,g) for walk,g in ranked if same_face(previous,g)
                       and not g.get('completed_box')
                       and not g.get('previous_attempts') and walk <= min(3.5,.20*(remaining-8))]
            if options:
                walk, goal = min(options, key=lambda row:row[0])
                return dict(goal, box_face_resume=True,
                            face_resume_count=previous.get('face_resume_count',0)+1,
                            local_batch=True)
        if self.box is not None and 0 <= elapsed-self.entered < RULES.box_visit_s:
            options = [(walk,g) for walk,g in ranked if g.get('batch_box') is not None
                       and math.dist(self.box,g['batch_box']) < .4 and not g.get('completed_box')
                       and (not g.get('previous_attempts')
                            or (g['previous_attempts'] <= 1 and not self.side_swept(g)))
                       and walk <= min(3.5,.20*(min(remaining,RULES.box_visit_s-(elapsed-self.entered))-8))]
            if options:
                tour = short_tour(options, grid, safe, min(remaining,RULES.box_visit_s-(elapsed-self.entered)))
                if tour is not None:
                    return dict(tour, local_batch=True, box_face_continuation=True)
        return None
