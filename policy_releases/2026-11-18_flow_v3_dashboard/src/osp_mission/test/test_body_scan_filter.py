import copy
import math
from types import SimpleNamespace

import pytest

from osp_mission.body_scan_filter import FOOTPRINT, BodyScanFilter, filter_ranges, inside_body, route_costmap_scans
from osp_mission.mission_outcome import outcome_status


def scan(ranges, angle=0., increment=.01):
    return SimpleNamespace(ranges=ranges, range_min=.15, range_max=12.,
        angle_min=angle, angle_increment=increment)


def test_measured_internal_returns_removed_and_external_returns_preserved():
    x, y = -.11456582644326924, -.13474396932633767
    distance = math.hypot(x+.04, y)
    angle = math.atan2(y, x+.04)-math.pi/2
    message = scan([distance, .30, .45], angle, 0.)
    values, removed = filter_ranges(message, -.04, 0., math.pi/2)
    assert removed == 1 and math.isinf(values[0])
    assert values[1:] == [.30, .45]
    assert message.ranges == [distance, .30, .45]


def test_points_in_front_of_or_outside_footprint_are_never_masked():
    for x, y in ((.19, 0.), (.25, 0.), (.3, 0.), (.13, .13), (.164, .068)):
        assert not inside_body(x, y)
    assert inside_body(-.115, -.134)


def test_invalid_and_no_return_measurements_remain_unchanged():
    values, removed = filter_ranges(scan([math.inf, math.nan, .1]), 0., 0., 0.)
    assert math.isinf(values[0]) and math.isnan(values[1]) and values[2] == .1
    assert removed == 0
    with pytest.raises(ValueError):
        filter_ranges(scan([.16]), math.nan, 0., 0.)


def test_missing_tf_withholds_planning_scan_instead_of_erasing_obstacles():
    def missing(*args, **kwargs):
        raise RuntimeError('missing TF')
    published = []
    owner = SimpleNamespace(tf=SimpleNamespace(lookup_transform=missing), tf_waits=0,
        get_parameter=lambda k:SimpleNamespace(value='base_link'),
        get_logger=lambda:SimpleNamespace(warning=lambda *a, **k:None),
        output=SimpleNamespace(publish=published.append))
    from builtin_interfaces.msg import Time
    message = scan([.2]); message.header=SimpleNamespace(frame_id='laser', stamp=Time())
    BodyScanFilter.on_scan(owner, message)
    assert not published and owner.tf_waits == 1 and message.ranges == [.2]


def test_costmap_routing_retains_raw_collision_monitor_and_input_config():
    import json
    layer = {'footprint':json.dumps(FOOTPRINT), 'obstacle_layer':{'scan':{'topic':'scan'}}}
    config = {key:{key:{'ros__parameters':copy.deepcopy(layer)}} for key in ('local_costmap','global_costmap')}
    config['collision_monitor']={'ros__parameters':{'scan':{'topic':'scan'}, 'enabled':True}}
    before=copy.deepcopy(config)
    routed=route_costmap_scans(config,'scan','scan_self_filtered')
    assert routed['collision_monitor']==before['collision_monitor'] and config==before
    for key in ('local_costmap','global_costmap'):
        assert routed[key][key]['ros__parameters']['obstacle_layer']['scan']['topic']=='scan_self_filtered'
    config['local_costmap']['local_costmap']['ros__parameters']['footprint']='[[0.1,0],[0,0.1],[-0.1,0],[0,-0.1]]'
    with pytest.raises(ValueError):
        route_costmap_scans(config,'scan','scan_self_filtered')


@pytest.mark.parametrize('reason,success', [('operator_stop',False), ('timeout',False),
                                          ('all_targets_found',True), ('bump_contact',False)])
def test_finished_mission_is_successful_only_for_confirmed_task_completion(reason,success):
    owner=SimpleNamespace(state='done', reason=reason, elapsed=12.3, started=1., now=lambda:999.)
    result=outcome_status(owner)
    assert result['task_success'] is success and result['termination_reason']==reason
    assert result['elapsed_seconds']==12.3


def test_running_clock_and_idle_state_are_not_frozen_or_completed():
    owner=SimpleNamespace(state='running', started=10., now=lambda:15.)
    assert outcome_status(owner)==dict(termination_reason='', task_success=False, elapsed_seconds=5.)
    owner.state='idle';owner.started=0.
    assert outcome_status(owner)['elapsed_seconds']==0.
