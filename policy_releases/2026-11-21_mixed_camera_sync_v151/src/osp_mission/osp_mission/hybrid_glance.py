"""Add a short bearing-only request between confirmed two-frame inspections."""
import math


class EdgeGlance:
    def __init__(self):
        self.last = -math.inf
        self.until = -math.inf
        self.recent = []
        self.last_stamp = -math.inf

    def update(self, now, stamp, camera, width, focal, cx, contours, confirmed):
        if stamp <= self.last_stamp:
            return None
        self.last_stamp = stamp
        if now < self.until:
            return None
        if confirmed is not None:
            self.last = now
            return confirmed
        if now-self.last < 12:
            return None
        self.recent = [row for row in self.recent if now-row[0]<90]
        for u,v,size in sorted(contours,key=lambda p:-p[2]):
            if min(u,width-1-u)>.20*width:
                continue
            angle = -math.atan((u-cx)/focal)
            bearing = camera[2]+angle
            if any(math.dist(camera[:2],pos)<.8 and
                   abs(math.remainder(bearing-last,2*math.pi))<math.radians(12)
                   for _,pos,last in self.recent):
                continue
            self.last = now
            self.until = now+.30
            self.recent.append((now,tuple(camera[:2]),bearing))
            return dict(action='turn',source='single',uv=[u,v],size=size,
                        yaw_delta_deg=max(-6.,min(6.,math.degrees(angle))),dwell_s=.30)
        return None
