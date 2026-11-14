import math
from types import SimpleNamespace
import numpy as np
import pytest
from scipy.ndimage import distance_transform_edt, label
from osp_mission.escape_connectivity import connect_start
from osp_mission.edge_policy import options
from osp_mission.exploration_memory import ExplorationMemory
from osp_mission.planning_diagnostics import explain
from osp_mission.transfer_noise import SensorStress


def candidate(x=2., y=2., yaw=0., gain=.4):
    return dict(x=x,y=y,yaw=yaw,features=np.array([2.,gain,3.,.1,0.]))


def test_repeated_high_frontier_cannot_bypass_memory_or_empty_fallback():
    memory=ExplorationMemory();goal=candidate()
    memory.selected(goal,(0,0,0));memory.completed(True,10)
    assert memory.filter([goal],(0,0,0),20)==[]
    assert memory.filter([goal],(0,0,0),100)==[]
    assert memory.filter([goal],(0,0,0),161)
    assert memory.filter([candidate(gain=.7)],(0,0,0),20)
    assert memory.filter([candidate(yaw=math.pi)],(0,0,0),20)
    assert memory.filter([candidate(x=5.)],(0,0,0),20)


def test_failed_navigation_has_bounded_backoff_without_recording_success():
    memory=ExplorationMemory();goal=candidate()
    for stamp in (10,26):
        memory.selected(goal,(0,0,0));memory.completed(False,stamp)
    assert not memory.views
    assert memory.filter([goal],(0,0,0),40)==[]
    assert memory.filter([goal],(0,0,0),57)
    memory.selected(goal,(0,0,0));memory.completed(True,58)
    assert not memory.failures


def test_memory_survives_map_shift_and_does_not_ban_corridor_transit():
    memory=ExplorationMemory();goal=candidate()
    memory.selected(goal,(0,0,0));memory.completed(True,10)
    shifted=candidate(x=0,y=0,yaw=-math.pi/2)
    assert not memory.filter([shifted],(2,2,math.pi/2),20)
    assert memory.filter([candidate(x=8.)],(0,0,0),20)
    memory.selected(dict(goal,approach_id=9),(0,0,0));memory.completed(False,20)
    assert not memory.failures


def test_body_clear_connector_can_leave_an_isolated_inflated_island():
    data=np.zeros((60,60),int);data[:,0]=100
    safe=(data==0)&(distance_transform_edt(data<=50)*.05>=.32)
    safe[:,6:9]=False;safe[30,6]=True
    connected=connect_start(data,.05,(30,6),safe)
    groups,_=label(connected)
    assert groups[30,6] and groups[30,6]==groups[30,15]
    assert np.all(data[connected&~safe]==0)
    assert np.min(distance_transform_edt(data==0)[connected&~safe]*.05)>=.189+.05/math.sqrt(2)+.01


@pytest.mark.parametrize('barrier',[-1,100])
def test_connector_never_clears_unknown_or_wall(barrier):
    data=np.zeros((60,60),int);data[:,8]=barrier
    safe=np.zeros_like(data,dtype=bool);safe[:,10:]=True;safe[30,4]=True
    connected=connect_start(data,.05,(30,4),safe)
    assert np.array_equal(connected,safe)


def test_negative_outside_start_is_not_truncated_into_first_map_cell():
    data=np.zeros((60,60),int);data[-1]=100
    assert options(data,.05,(0,0),(-.001,1.5))[0]==[]
    assert explain(data,.05,(0,0),(-.001,1.5),0,0)['reason']=='start_outside_map'
    assert explain(data,.05,(0,0),(1.5,1.5),2,0)['reason']=='viewpoint_retry_cooldown'


def test_noisy_lidar_preserves_invalids_stamp_and_reproducibility():
    scan=SimpleNamespace(ranges=[1.]*640+[float('nan'),float('inf'),0.],range_min=.1,range_max=12.,header={'stamp':123})
    a=SensorStress('moderate',7);b=SensorStress('moderate',7)
    for _ in range(80):
        x=a.scan(scan);y=b.scan(scan)
        assert (x is None)==(y is None)
        if x is None:continue
        assert x.header==scan.header
        assert np.allclose(x.ranges,y.ranges,equal_nan=True)
        assert math.isnan(x.ranges[-3]) and math.isinf(x.ranges[-2]) and x.ranges[-1]==0
        assert x.ranges[:640]!=scan.ranges[:640]
    assert a.dropped>0
    assert all(v==1. for v in scan.ranges[:640])
    assert SensorStress('none').scan(scan) is scan


def test_encoder_stress_preserves_psd_covariance():
    from geometry_msgs.msg import TwistWithCovarianceStamped
    message=TwistWithCovarianceStamped();message.twist.twist.linear.x=.2;message.twist.twist.angular.z=.3
    message.twist.covariance=np.eye(6).ravel().tolist()
    result=SensorStress('moderate').twist(message)
    assert result.twist.twist.linear.x==pytest.approx(.206)
    assert result.twist.twist.angular.z==pytest.approx(.294)
    assert np.linalg.eigvalsh(np.array(result.twist.covariance).reshape(6,6)).min()>0
    assert message.twist.twist.linear.x==.2
