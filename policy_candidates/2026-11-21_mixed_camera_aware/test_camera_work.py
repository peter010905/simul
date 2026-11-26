from types import SimpleNamespace
import pytest
from camera_work import ExposureQueue


def test_newest_ready_exposure_survives_tf_lag_longer_than_frame_period():
    queue=ExposureQueue()
    for stamp in (1.,1.066,1.133):queue.offer(stamp,{},stamp,stamp)
    row=queue.take(1.14,lambda row:row.stamp<=1.07)
    assert row.message==1.066
    assert [item.stamp for item in queue.rows]==[1.133]
    assert queue.take(1.18,lambda row:True).stamp==1.133


def test_queue_is_bounded_and_drops_old_work_before_tf_retry():
    queue=ExposureQueue(capacity=3)
    for step in range(20):queue.offer(step,{},1+step*.02,1+step*.02)
    assert len(queue.rows)==3
    assert queue.counts['replaced']==17
    assert queue.take(2.,lambda row:True) is None
    assert queue.counts['expired']==3


def test_future_duplicate_and_clock_reset_cannot_create_old_surface_evidence():
    queue=ExposureQueue()
    assert not queue.offer(None,None,11.,10.)
    assert queue.offer('old',None,10.,10.)
    assert not queue.offer('duplicate',None,10.,10.)
    assert queue.offer('new_epoch',None,1.,1.)
    assert queue.take(1.01,lambda row:True).message=='new_epoch'
    assert queue.counts['clock_resets']==1


@pytest.mark.parametrize('stamp,now',[(float('nan'),1.),(1.,float('inf')),(0.,0.)])
def test_invalid_capture_clock_never_enters_queue(stamp,now):
    queue=ExposureQueue()
    assert not queue.offer(None,None,stamp,now)
    assert not queue.rows
