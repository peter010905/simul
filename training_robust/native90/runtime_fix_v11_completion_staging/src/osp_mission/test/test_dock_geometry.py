import math
import numpy as np
from osp_mission.dock_geometry import mark,points


def test_station_is_occupied_in_a_rotated_map_without_blocking_escape():
    grid=np.zeros((100,100),dtype=np.int8)
    mark(grid,.02,(-1.,-1.),(.2,0.,math.pi))
    # Rotating the local x=-.045 box by pi puts its centre at x=.245.
    assert grid[50,62]==100
    assert grid[50,25]==0  # undocked robot at x=-.5 can leave
    assert grid[70,62]==0  # a route around the station remains free
    assert .005<points()[:,2].min()<.19  # below the horizontal LiDAR
