"""Stop on motor stall and require fresh clear hazards and measured rest to recover."""
import math


class StallPause:
    def __init__(self):
        self.paused = False
        self.active_since = None
        self.clear_since = None
        self.still_since = None
        self.updated = -math.inf
        self.critical = False
        self.episodes = 0
        self.recovered = 0

    def update(self, types, now):
        if not math.isfinite(now) or now < self.updated:
            return
        types = set(types)
        self.updated = now
        if types & {2, 4}:
            self.critical = True
            self.paused = True
        if 3 in types:
            if self.active_since is None:
                self.active_since = now
                self.episodes += 1
            self.paused = True
        else:
            self.active_since = None
        if types & {1, 2, 3, 4}:
            self.clear_since = None
            self.still_since = None
        elif self.paused and self.clear_since is None:
            self.clear_since = now

    def fatal(self, now):
        return self.critical or (self.active_since is not None and now-self.active_since >= 2.)

    def resume(self, now, stationary):
        if not self.paused or self.fatal(now):
            return False
        if not stationary or not 0 <= now-self.updated <= .6:
            self.still_since = None
            return False
        if self.still_since is None:
            self.still_since = now
        if self.clear_since is None or now-self.clear_since < 1. or now-self.still_since < .5:
            return False
        self.paused = False
        self.recovered += 1
        self.clear_since = self.still_since = None
        return True

    def report(self):
        return dict(paused=self.paused, episodes=self.episodes, recovered=self.recovered)
