"""Stop-to-stop angular motion bounded by speed and acceleration."""

import math


class RotationProfile:
    def __init__(self,angle,max_speed,acceleration):
        if not all(math.isfinite(v) for v in (angle,max_speed,acceleration)) or max_speed<=0 or acceleration<=0:
            raise ValueError('Finite angle and positive motion limits are required')
        self.angle=abs(angle);self.acceleration=acceleration
        self.peak_speed=min(max_speed,math.sqrt(self.angle*acceleration))
        self.ramp=self.peak_speed/acceleration
        self.cruise=max(0.,self.angle-self.peak_speed*self.ramp)/self.peak_speed if self.peak_speed else 0.
        self.duration=2*self.ramp+self.cruise

    def position(self,elapsed):
        t=min(self.duration,max(0.,elapsed))
        if t<self.ramp:return .5*self.acceleration*t*t
        if t<self.ramp+self.cruise:
            return .5*self.peak_speed*self.ramp+self.peak_speed*(t-self.ramp)
        return self.angle-.5*self.acceleration*(self.duration-t)**2
