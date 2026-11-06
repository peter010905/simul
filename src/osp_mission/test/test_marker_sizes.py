import copy
import pytest
from osp_mission.marker_sizes import randomize_sizes,face_size_limit


def scene():
    return dict(seed=10,start=[1,1,0],obstacles=[dict(type='box',sx=1.,sy=.4,height=1.,yaw=0.)],
        targets=[dict(id=i,x=3+i,y=2.,yaw=0.,size=.2,mount='wall') for i in range(3)])


def test_size_seed_changes_only_size_and_metadata():
    cfg=scene();original=copy.deepcopy(cfg)
    a=randomize_sizes(cfg,100);b=randomize_sizes(cfg,101)
    assert a==randomize_sizes(cfg,100) and cfg==original
    assert [t['size'] for t in a['targets']]!=[t['size'] for t in b['targets']]
    assert a['start']==cfg['start'] and a['obstacles']==cfg['obstacles']
    for x,y in zip(a['targets'],cfg['targets']):
        assert {k:v for k,v in x.items() if k not in ('size','size_sampling_bounds_m')}=={k:v for k,v in y.items() if k!='size'}
    assert sorted(int((t['size']-.08)/(.22/3)) for t in a['targets'])==[0,1,2]


def test_obstacle_face_and_white_backing_fit():
    cfg=scene();cfg['targets'][0].update(mount='obstacle',carrier_obstacle=0)
    a=randomize_sizes(cfg,20,maximum=.6)
    assert a['targets'][0]['size']<=face_size_limit(cfg,cfg['targets'][0])
    assert a['targets'][0]['size']*1.25<cfg['obstacles'][0]['sy']
    with pytest.raises(ValueError):randomize_sizes(cfg,20,minimum=.4,maximum=.6)


def test_large_size_respects_short_carrier_top_at_fixed_marker_height():
    cfg=scene();cfg['obstacles'][0].update(sy=2.,height=.65)
    cfg['targets'][0].update(mount='obstacle',carrier_obstacle=0)
    result=randomize_sizes(cfg,20,maximum=.6)
    size=result['targets'][0]['size']
    assert .45+1.25*size/2<.65


@pytest.mark.parametrize('low,high',[(.04,.3),(.3,.1),(.1,float('nan'))])
def test_invalid_size_ranges_rejected(low,high):
    with pytest.raises(ValueError):randomize_sizes(scene(),1,low,high)
