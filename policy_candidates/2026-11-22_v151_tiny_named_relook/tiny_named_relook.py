"""Corroborate subthreshold decoded prints for one short, range-free relook."""
import math


def aligned_turn(ray, exposure_yaw, current):
    """Return a six-degree step only if it still reduces the off-axis angle."""
    before = math.remainder(ray - exposure_yaw, math.tau)
    after = math.remainder(ray - current, math.tau)
    step = math.radians(6)
    if before * after <= 0 or not step <= abs(after) <= math.radians(55):
        return None
    return math.copysign(step, after)


class TinyNamedRelook:
    def __init__(self):
        self.previous = {}
        self.last_stamp = -math.inf
        self.last_glance = None

    def near_pass_covered(self, now, origin, ray):
        if self.last_glance is None:
            return False
        t,p,b = self.last_glance
        return (0 <= now-t < 15 and math.dist(origin,p)<.8
                and abs(math.remainder(ray-b,math.tau))<math.radians(12))

    def offer(self, stamp, now, origin, yaw, hits, known, ledger):
        if (len(origin) != 2 or not all(map(math.isfinite, [*origin, yaw, now, stamp]))
                or not -.1 <= now-stamp <= .5 or stamp <= self.last_stamp):
            return None
        self.last_stamp = stamp
        self.previous = {i:r for i,r in self.previous.items() if stamp-r[0] <= .85}
        if len(hits) > 4:
            self.previous.clear()
            return None
        evidence = {}
        for ident, ray, size, vertical in hits:
            if (type(ident) is not int or not 0 <= ident < 1024 or ident in known
                    or not all(map(math.isfinite, (ray, size, vertical)))
                    or not 8 <= size < 10 or abs(vertical) > 1):
                continue
            evidence.setdefault(ident, []).append((ray,size,vertical))
        # An alternating payload at the same image location breaks the sequence.
        conflicts = set()
        for old_id,old in self.previous.items():
            if math.dist(old[1],origin)>.22:
                continue
            for ident,group in evidence.items():
                if ident!=old_id and any(abs(math.remainder(old[2]-b,math.tau))<math.radians(2.2)
                                          and abs(old[4]-y)<.10 for b,_,y in group):
                    conflicts.update((old_id,ident))
        for ident in conflicts:
            self.previous.pop(ident,None)
        matches = []
        for ident, group in evidence.items():
            old = self.previous.pop(ident,None)
            if len(group) != 1 or ident in conflicts:
                continue
            ray,size,v = group[0]
            # Another plausible ID at the same location makes this exposure ambiguous.
            if any(abs(math.remainder(ray-b,math.tau)) < math.radians(2.2) and abs(v-y)<.10
                   for other,rows in evidence.items() if other!=ident for b,_,y in rows):
                continue
            self.previous[ident] = (stamp,tuple(origin),ray,size,v)
            if (old is None or not .15 <= stamp-old[0] <= .85 or math.dist(old[1],origin)>.22
                    or abs(math.remainder(old[2]-ray,math.tau))>math.radians(2.2)
                    or not .80 <= size/old[3] <= 1.25 or abs(v-old[4])>.10):
                continue
            if aligned_turn(ray,yaw,yaw) is not None:
                matches.append((ident,ray))
        if len(matches)!=1 or len(ledger)>=2 or (ledger and now-ledger[-1][0]<30):
            return None
        ident,ray = matches[0]
        if any(math.dist(p,origin)<.8 and abs(math.remainder(b-ray,math.tau)) < math.radians(12)
               for _,p,b in ledger):
            return None
        return ident,(now,tuple(origin),ray,aligned_turn(ray,yaw,yaw))
