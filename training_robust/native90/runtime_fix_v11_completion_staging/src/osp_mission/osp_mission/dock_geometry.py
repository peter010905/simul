"""The known charging station is lower than the robot's horizontal LiDAR."""
import math
import numpy as np


def points():
    # Upstream Create 3 dock collision: x=-.045, size .04 x .13 x .10.
    # Include 3 cm localization margin. These are a known static keepout,
    # not a simulated sensor observation or an unknown scene obstacle.
    x,y=np.meshgrid(np.linspace(-.095,.005,6),np.linspace(-.095,.095,11))
    return np.column_stack((x.ravel(),y.ravel(),np.full(x.size,.05)))


def mark(data,resolution,origin,pose):
    x,y,a=pose
    rows,cols=np.indices(data.shape)
    dx=origin[0]+(cols+.5)*resolution-x
    dy=origin[1]+(rows+.5)*resolution-y
    local_x=dx*math.cos(a)+dy*math.sin(a)
    local_y=-dx*math.sin(a)+dy*math.cos(a)
    margin=resolution/2
    data[(local_x>=-.095-margin)&(local_x<=.005+margin)&(abs(local_y)<=.095+margin)]=100
    return data
