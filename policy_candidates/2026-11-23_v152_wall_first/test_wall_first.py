from osp_mission.wall_first import WallFirst


WALL = {'r32_kind': 'wall_surface'}


def face(center=None, attempts=0):
    return {'batch_box': center, 'previous_attempts': attempts, 'r32_kind': 'interior_surface'}


def test_detour_stays_with_same_object_and_resumes_wall():
    patrol = WallFirst()
    box = face((1., 0.))
    nearby = face((0., 1.))
    patrol.dispatched(WALL, 5.)
    assert patrol.probe_wall
    first = patrol.obstacle([(1., box)], (0., 0.), 10., 90.)
    patrol.dispatched(first, 10.)
    assert not patrol.probe_wall
    second = patrol.obstacle([(1., nearby), (1., box)], (0., 0.), 20., 80.)
    assert second['patrol_inspection'] == [1., 0.]
    assert patrol.obstacle([(1., nearby)], (0., 0.), 30., 70.) is None
    patrol.dispatched(WALL, 30.)
    assert not patrol.probe_wall
    assert patrol.obstacle([(1., box)], (0., 0.), 40., 60.) is None
    patrol.dispatched(WALL, 45.)
    assert patrol.probe_wall


def test_only_live_reachable_nearby_faces_and_bounded_detour():
    patrol = WallFirst()
    box = face((1., 0.))
    candidates = [(4., box), (1., face((3., 0.))), (1., face((1., 0.), 1)), (1., face())]
    assert patrol.obstacle(candidates, (0., 0.), 10., 100.) is None
    assert patrol.obstacle([(1., box)], (0., 0.), 10., 8.) is None
    goal = patrol.obstacle([(1., box)], (0., 0.), 10., 100.)
    for time in (10., 15., 20., 25.):
        patrol.dispatched(goal, time)
    assert patrol.obstacle([(1., box)], (0., 0.), 30., 100.) is None


def test_marker_approach_preserves_active_obstacle_and_deadline():
    patrol = WallFirst()
    box = face((1., 0.))
    goal = patrol.obstacle([(1., box)], (0., 0.), 10., 100.)
    patrol.dispatched(goal, 10.)
    patrol.dispatched({'approach_id': 42}, 15.)
    assert patrol.object_center == [1., 0.]
    assert patrol.obstacle([(1., box)], (0., 0.), 55., 100.) is None


def test_surface_goals_cannot_indefinitely_delay_space_exploration():
    patrol = WallFirst()
    assert not patrol.expansion_due(20.)
    patrol.dispatched(WALL, 5.)
    assert not patrol.expansion_due(49.9)
    assert patrol.expansion_due(50.)
    patrol.dispatched(WALL, 12.)
    patrol.dispatched(WALL, 25.)
    assert patrol.expansion_due(26.)
    patrol.dispatched({'r32_kind': 'interior_frontier'}, 27.)
    assert not patrol.expansion_due(100.)
