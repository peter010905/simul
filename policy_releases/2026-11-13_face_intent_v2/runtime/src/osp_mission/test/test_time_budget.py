"""Deadline handling must preserve successful camera observations."""

import math
import numpy as np

from osp_mission.fast_sim import SearchEnv
from osp_mission.time_budget import choose


def test_deadline_avoids_unfinishable_goal_but_keeps_the_unlimited_choice():
    far=dict(scan_angle=2*math.pi/3,features=np.array([100.,0.,10.,0.,0.]))
    near=dict(scan_angle=2*math.pi/3,features=np.array([1.,0.,.5,0.,0.]))
    weights=np.ones(5)
    assert choose([far,near],weights,float('inf')) is far
    assert choose([far,near],weights,12.)['features'][2]==.5
    assert choose([far,near],weights,1.) is None


def test_discovery_during_last_recovery_spin_counts_before_deadline():
    cfg=dict(seed=1,start=[2.,2.,0.],obstacles=[],targets=[
        dict(id=ident,x=3.,y=2.,yaw=math.pi,size=.3) for ident in range(3)])
    env=SearchEnv(cfg,seed=1,camera_width=960,camera_hz=5,detection_dropout=0.)
    env.elapsed=599.;env.deadline=600.
    env.confirmations={ident:2 for ident in range(3)}
    env.last_confirmation={ident:598.7 for ident in range(3)}
    env.options=lambda *args:([],None)
    env.choose=lambda *args:None
    env.step(np.ones(9),deadline=600.)
    assert len(env.found)==3 and env.elapsed<600.


def test_faster_spin_preserves_configured_camera_frame_rate():
    cfg=dict(seed=1,start=[2.,2.,0.],obstacles=[],targets=[])
    env=SearchEnv(cfg,seed=1,camera_width=960,camera_hz=5.)
    env.turn_speed=1.;frames=[]
    env.detect=lambda:frames.append(env.elapsed)
    env.rotate(1.,deadline=2.)
    np.testing.assert_allclose(frames,[.2,.4,.6,.8,1.],atol=1e-9)
