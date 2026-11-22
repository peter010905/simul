import math
from types import SimpleNamespace as NS
import numpy as np
from osp_mission.wall_route import WallLook, penalize_routes


def test_center_side_wall_but_skip_gap_and_recent_observation():
    a=np.linspace(-math.pi,math.pi,641)
    r=np.full(len(a),np.inf)
    wall=(a>1.)&(a<2.)
    r[wall]=.8/np.sin(a[wall])
    scan=NS(ranges=r,angle_min=a[0],angle_increment=a[1]-a[0],range_min=.12,range_max=10.)
    w=WallLook()
    point=w.select(scan,(0.,0.,0.),(0.,0.,0.),1.1,15.)
    assert point and abs(point[1]-.8)<.01
    w.recent.append((15.,point,math.atan2(point[1],point[0])))
    assert w.select(scan,(0.,0.,0.),(0.,0.,0.),1.1,30.) is None
    scan.ranges[::3]=np.inf
    assert w.select(scan,(0.,0.,0.),(0.,0.,0.),1.1,150.) is None


def test_alternative_branch_beats_traversed_route_and_common_exit_remains():
    previous=np.array([-1,0,1,2,0,4,5])
    goals=[{'cell':(0,3),'features':np.zeros(5)}, {'cell':(1,2),'features':np.zeros(5)}]
    memory={(x,0):{'entries':2} for x in range(4)}
    penalize_routes(goals,previous,(2,4),.6,(0.,0.),(0.,0.,0.),memory)
    assert goals[0]['features'][4]>goals[1]['features'][4]==0.



def test_curved_post_and_second_view_not_suppressed():
    angles=np.linspace(-math.pi,math.pi,721)
    rays=np.column_stack((np.cos(angles),np.sin(angles)))
    center=np.array([0.,1.2]); radius=.15
    dot=rays@center; disc=dot**2-center@center+radius**2
    ranges=np.full(len(angles),np.inf)
    hit=(dot>0)&(disc>=0)
    ranges[hit]=dot[hit]-np.sqrt(disc[hit])
    scan=NS(ranges=ranges,angle_min=angles[0],angle_increment=angles[1]-angles[0],range_min=.1,range_max=10.)
    survey=WallLook()
    target=survey.select(scan,(0.,0.,0.),(0.,0.,0.),1.1,30.)
    assert target is not None and 1.0<target[1]<1.15
    survey.recent.append((25.,target,-math.pi/2))
    assert survey.select(scan,(0.,0.,0.),(0.,0.,0.),1.1,30.) is not None
