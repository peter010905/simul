from copy import deepcopy

from run_floor_pose_regression import floor_gate


def report():
    names=('cliff_front_left','cliff_front_right','cliff_side_left','cliff_side_right')
    return dict(samples={'hazard_frames':30,**{n:10 for n in names}},hazards={},
                cliff_scans={n:dict(values=10,finite=10,infinite=0) for n in names})


def test_all_four_floor_streams_and_hazard_stream_are_required():
    measured=report()
    assert floor_gate(measured)
    for name in measured['cliff_scans']:
        missing=deepcopy(measured)
        del missing['cliff_scans'][name]
        assert not floor_gate(missing)
    measured['samples']['hazard_frames']=0
    assert not floor_gate(measured)


def test_infinite_rays_or_even_one_cliff_event_block_training():
    measured=report()
    measured['cliff_scans']['cliff_front_left']['finite']=9
    measured['cliff_scans']['cliff_front_left']['infinite']=1
    assert not floor_gate(measured)
    measured=report()
    measured['hazards']['2:base_link']=1
    assert not floor_gate(measured)
