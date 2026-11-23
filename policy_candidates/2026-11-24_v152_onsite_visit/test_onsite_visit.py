from osp_mission.regional_surface import RegionalSurface


def face(x, key):
    return dict(r32_kind='interior_surface', region_anchor=(x, 0.), region_face=key,
                joint_continuation=False)


def test_long_transit_does_not_use_local_observation_time():
    visit = RegionalSurface()
    visit.position(0., 0., 225.)
    visit.dispatched(face(10., 'a'), 227.4)
    visit.position(7., 0., 252.)
    assert visit.entered is None and not visit.can_continue(252.)
    visit.position(8., 0., 262.2)
    visit.dispatched(face(10.3, 'b'), 273.9)
    assert visit.entered == 262.2 and visit.can_continue(283.9)
    assert not visit.can_continue(317.2)
    visit.dispatched(face(10.5, 'c'), 318.)
    assert visit.entered == 262.2 and not visit.can_continue(318.)


def test_stop_budget_interruption_release_and_new_region():
    visit = RegionalSurface()
    visit.position(0., 0., 1.)
    for n in range(4):
        visit.dispatched(face(.1*n, n), 2.+n)
    assert visit.entered == 2. and not visit.can_continue(7.)
    visit.dispatched(dict(r32_kind='flow_or_marker'), 9.)
    assert visit.attempted == {0, 1, 2, 3}
    visit.dispatched(dict(r32_kind='interior_frontier'), 10.)
    assert visit.anchor is None
    visit.dispatched(face(7., 'new'), 20.)
    assert visit.stops == 1 and visit.entered is None
    visit.position(6.2, .1, 26.)
    assert visit.entered == 26. and visit.can_continue(27.)
