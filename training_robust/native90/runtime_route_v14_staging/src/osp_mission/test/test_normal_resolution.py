import math
import numpy as np
import pytest
from osp_mission.directional_policy import normals


@pytest.mark.parametrize('resolution',[.05,.1,.2])
@pytest.mark.parametrize('angle',[0.,math.pi/4,math.pi/2])
def test_thin_wall_has_a_normal_axis_at_native_slam_resolution(resolution,angle):
    size=int(8/resolution)+1;middle=size//2
    yy,xx=np.indices((size,size));x=(xx-middle)*resolution;y=(yy-middle)*resolution
    nx,ny=math.cos(angle),math.sin(angle)
    grid=np.where(abs(nx*x+ny*y)<resolution*.55,100,0).astype(np.int8)
    normal_y,normal_x=normals(grid,resolution)
    alignment=abs(normal_x[middle,middle]*nx+normal_y[middle,middle]*ny)
    assert alignment>.99
