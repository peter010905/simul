"""Bound post-contact motion and schedule short camera observations."""
import math


class ContactInspection:
    def __init__(self):
        self.phase = ''
        self.action = ''
        self.pressed = False
        self.hazard_time = -math.inf
        self.release_time = -math.inf
        self.next_try = 0.
        self.side = 1.
        self.scan_sign = 1.
        self.last_look = -math.inf
        self.look_position = None
        self.position = None
        self.bearing = None
        self.last_bearing = {}

    def hazard(self, frames, now):
        self.hazard_time = now
        was_pressed = self.pressed
        self.pressed = bool(frames)
        if self.pressed and not was_pressed:
            self.phase = 'retreat'
            self.bearing = None
            if any('left' in f for f in frames):
                self.side = -1.
            elif any('right' in f for f in frames):
                self.side = 1.
            else:
                self.side *= -1.
            return True
        if was_pressed and not self.pressed:
            self.release_time = now
        return False

    def recovery(self, now, rear_clear, backup_ready, spin_ready):
        if not self.phase:
            return ''
        if self.pressed or now-self.hazard_time > .7 or now-self.release_time < .5 or now < self.next_try:
            return 'hold'
        if self.phase == 'retreat' and rear_clear and backup_ready:
            return 'backup'
        return 'spin' if spin_ready else 'hold'

    def result(self, success, now):
        action, self.action = self.action, ''
        if action == 'backup':
            self.phase = 'turn'
        elif action == 'spin':
            self.phase = '' if success else 'retreat'
            self.next_try = now + (0. if success else 2.)
        if self.pressed:
            self.phase = 'retreat'

    def due(self, now):
        return (self.position is not None and self.look_position is not None and
                now-self.last_look >= 25. and math.dist(self.position, self.look_position) >= 2.)

    def looked(self, now):
        self.look_position = self.position
        self.last_look = now

    def survey_angle(self):
        angle = self.scan_sign * math.radians(45)
        self.scan_sign *= -1
        return angle


class BearingCue:
    """Require three recent, spatially consistent image bearings before looking."""
    def __init__(self):
        self.history = {}
        self.published = {}

    def observe(self, ident, stamp, angle):
        previous, last, count = self.history.get(ident, (-math.inf, angle, 0))
        if not math.isfinite(angle) or stamp <= previous:
            return False
        count = count+1 if stamp-previous <= .5 and abs(angle-last) <= .12 else 1
        self.history[ident] = stamp, angle, count
        if count < 3 or stamp-self.published.get(ident, -math.inf) < 15.:
            return False
        self.published[ident] = stamp
        return True
