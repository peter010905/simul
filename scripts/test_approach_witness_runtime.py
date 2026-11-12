import numpy as np

from osp_mission.marker_approach import MarkerApproach


def evidence(stamp,**fields):
    return dict(stamp=stamp,id=44,map_point=[1.38,0.,.45],odom_point=[1.38,0.,.45],
        camera_point=[0.,0.,1.38],lidar_camera_point=[0.,0.,1.39],
        min_edge_pixels=30,observation_frame='map',**fields)


def observe(approach,stamp,**kwargs):
    approach.observe(evidence(stamp),(0,0),stamp,confirmed=True,odom_robot=(0,0),**kwargs)


def test_three_readable_confirmed_near_frames_survive_map_frame_correction():
    approach=MarkerApproach();approach.reconcile_targets({44:(1.38,0)})
    for stamp in [1.,1.2]:observe(approach,stamp)
    assert not approach.verified(44)
    observe(approach,1.4)
    assert approach.verified(44,target=(8.,3.))
    assert approach.report()['verified_ids']==[44]
    assert len(approach.report()['range_witness_evidence'][44]['observations'])==3


def test_provisional_unconfirmed_or_inconsistent_ranges_never_grant_approach():
    for mode in ['unconfirmed','provisional','small','range_conflict','far']:
        approach=MarkerApproach();approach.reconcile_targets({44:(1.38,0)})
        for stamp in [1.,1.2,1.4,1.6]:
            e=evidence(stamp)
            if mode=='provisional':e['provisional']=True
            if mode=='small':e['min_edge_pixels']=17
            if mode=='range_conflict':e['lidar_camera_point']=[0.,0.,1.6]
            if mode=='far':e['odom_point']=[1.6,0.,.45]
            approach.observe(e,(0,0),stamp,confirmed=mode!='unconfirmed',odom_robot=(0,0))
        assert not approach.verified(44)


def test_missing_odom_duplicate_or_large_time_gap_cannot_accumulate_three():
    approach=MarkerApproach()
    for stamp in [1.,1.,1.,3.,3.2]:observe(approach,stamp)
    assert not approach.verified(44)
    for stamp in [4.,4.2,4.4]:approach.observe(evidence(stamp),(0,0),stamp,confirmed=True)
    assert not approach.verified(44)


def test_identity_retraction_clears_physical_witness_and_all_old_evidence():
    approach=MarkerApproach();approach.reconcile_targets({44:(1.38,0)})
    for stamp in [1.,1.2,1.4]:observe(approach,stamp)
    assert approach.verified(44)
    approach.reconcile_targets({})
    assert not approach.verified(44)
    approach.reconcile_targets({44:(10.,10.)})
    assert not approach.verified(44)


def test_old_pose_only_credit_keeps_its_landmark_consistency_check():
    approach=MarkerApproach();approach.observe_pose({44:(1.2,0)},(0,0),1.,1.)
    assert approach.verified(44,(1.2,0))
    assert not approach.verified(44,(8,8))


def test_replayed_native_close_ranges_are_enough_but_do_not_bypass_true_radius():
    approach=MarkerApproach();approach.reconcile_targets({910:(0,0)})
    for stamp,distance in [(50.,1.398),(50.2,1.390),(50.4,1.382)]:
        e=evidence(stamp);e.update(id=910,odom_point=[distance,0.,.45],
            camera_point=[0.,0.,distance],lidar_camera_point=[0.,0.,distance+.01])
        approach.observe(e,(0,0),stamp,confirmed=True,odom_robot=(0,0))
    assert approach.verified(910)
    assert max(r['distance_m'] for r in approach.range_witnesses[910]['observations'])<=1.4
