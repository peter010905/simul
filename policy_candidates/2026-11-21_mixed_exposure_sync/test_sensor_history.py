from types import SimpleNamespace as NS
import math
from sensor_history import SensorHistory


def message(stamp,frame='camera',width=480,encoding='32FC1'):
    sec=math.floor(stamp)
    return NS(header=NS(stamp=NS(sec=sec,nanosec=round((stamp-sec)*1e9)),frame_id=frame),
              width=width,height=width,encoding=encoding)


def test_delayed_rgb_uses_its_depth_instead_of_latest_depth():
    history=SensorHistory()
    expected=message(10.0)
    for row in (expected,message(10.1),message(10.2),message(10.3)):history.depth(row)
    history.scan(message(9.98,'laser'));history.scan(message(10.03,'laser'))
    scan,depth=history.select(message(10.0))
    assert depth is expected and scan.header.stamp.sec==9
    assert abs(history.report()['latest_scan_skew_s']-.02)<1e-8


def test_unregistered_depth_future_laser_and_wrong_resolution_cannot_supply_range():
    history=SensorHistory()
    for row in (message(10.,'other_optical'),message(10.,width=640),message(10.,encoding='8UC1')):history.depth(row)
    history.scan(message(10.1,'laser'));history.scan(message(9.7,'laser'))
    assert history.select(message(10.0))==(None,None)


def test_sensor_memory_is_bounded_and_old_epoch_samples_are_not_reused():
    history=SensorHistory()
    for index in range(100):history.scan(message(10+index*.01,'laser'));history.depth(message(10+index*.01))
    assert len(history.scans)==64 and len(history.depths)==8
    assert history.select(message(1.))==(None,None)
