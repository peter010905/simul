import numpy as np
from osp_mission.marker_relook import VisualRange
from osp_mission.camera_multiview import CameraVisualRange


POINT=np.array([3.,0.,.45])


def observe(model,stamp,origin):
    origin=np.array(origin,dtype=float)
    direction=POINT-origin;distance=np.linalg.norm(direction)
    return model.observe(7,stamp,origin,direction/distance,80./distance)


def test_three_moving_5fps_frames_are_geometrically_usable():
    old,new=VisualRange(),CameraVisualRange()
    for start,side in ((1.,-.8),(3.,.4)):
        for step in range(3):
            origin=(0.,side+step*.05,.45)
            before=observe(old,start+step*.2,origin)
            after=observe(new,start+step*.2,origin)
    assert before is None
    np.testing.assert_allclose(after,POINT,atol=1e-8)


def test_older_valid_parallax_is_not_hidden_by_latest_invalid_viewpoint():
    old,new=VisualRange(),CameraVisualRange()
    for start,side in ((1.,-.8),(3.,-.36),(5.,0.)):
        for step in range(3):
            before=observe(old,start+step*.1,(0.,side,.45))
            after=observe(new,start+step*.1,(0.,side,.45))
    assert before is None
    np.testing.assert_allclose(after,POINT,atol=1e-8)


def test_60_second_view_retention_is_independent_of_15fps_frame_count():
    old,new=VisualRange(),CameraVisualRange()
    for step in range(630):
        stamp=1.+step/15
        side=-.8 if step<8 else .4
        before=observe(old,stamp,(0.,side,.45))
        after=observe(new,stamp,(0.,side,.45))
    before=observe(old,43.2,(0.,.4,.45))
    after=observe(new,43.2,(0.,.4,.45))
    assert before is None
    np.testing.assert_allclose(after,POINT,atol=1e-8)
    assert len(new.history[7])<=16 and len(new.views[7])<=320


def test_missing_parallax_never_turns_into_a_range_measurement():
    new=CameraVisualRange()
    for step in range(100):
        assert observe(new,1.+step*.1,(0.,.4,.45)) is None


def test_old_viewpoints_expire_and_clock_reset_clears_both_buffers():
    new=CameraVisualRange()
    for step in range(5):observe(new,1.+step*.1,(0.,-.8,.45))
    for step in range(5):result=observe(new,70.+step*.1,(0.,.4,.45))
    assert result is None
    observe(new,1.,(0.,0.,.45))
    assert not new.views
