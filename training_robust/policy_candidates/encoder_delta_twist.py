"""Causal body velocity from consecutive encoder odometry poses,never world truth."""
import copy
import math


class EncoderDelta:
    def __init__(self,maximum_gap_s=.25):
        self.previous=None;self.maximum_gap_s=maximum_gap_s

    def update(self,message):
        h=message.header;t=h.stamp.sec+h.stamp.nanosec/1e9
        p=message.pose.pose.position;q=message.pose.pose.orientation
        values=[t,p.x,p.y,q.x,q.y,q.z,q.w]
        if not all(math.isfinite(value) for value in values):return None
        norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        if not .9<=norm<=1.1:return None
        x,y,z,w=q.x/norm,q.y/norm,q.z/norm,q.w/norm
        yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        current=(t,p.x,p.y,yaw,h.frame_id,message.child_frame_id)
        old=self.previous
        if old is not None and t<=old[0]:return None
        self.previous=current
        if old is None or current[4:]!=old[4:]:return None
        dt=t-old[0]
        if not .001<=dt<=self.maximum_gap_s:return None
        heading=old[3]+.5*((yaw-old[3]+math.pi)%(2*math.pi)-math.pi)
        dx,dy=p.x-old[1],p.y-old[2];c,s=math.cos(heading),math.sin(heading)
        result=copy.deepcopy(message)
        result.twist.twist.linear.x=(c*dx+s*dy)/dt
        result.twist.twist.linear.y=(-s*dx+c*dy)/dt
        return result
