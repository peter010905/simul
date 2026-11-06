from types import SimpleNamespace
import math
import numpy as np
from osp_mission.scan_safety import rear_clear


def scan():
    return SimpleNamespace(ranges=np.full(640, 3.),angle_min=-math.pi,
                           angle_increment=2*math.pi/640,range_min=.12,range_max=12.)


def hit(s, angle, distance):
    s.ranges[int(round((angle-s.angle_min)/s.angle_increment))%len(s.ranges)]=distance


def test_mount_rotation_and_offset_put_obstacle_in_actual_rear_corridor():
    s=scan();hit(s,math.pi/2,.30)
    assert rear_clear(s,(0.,0.,0.))
    assert not rear_clear(s,(0.,0.,math.pi/2))
    # A sensor behind base_link sees an obstacle .5 m away that is actually
    # beyond the short retreat; a forward-mounted one can see it inside it.
    s=scan();hit(s,math.pi,.50)
    assert rear_clear(s,(-.10,0.,0.))
    assert not rear_clear(s,(.10,0.,0.))


def test_front_contact_allows_retreat_but_rear_side_contact_does_not():
    s=scan();hit(s,0.,.19)
    assert rear_clear(s,(0.,0.,0.))
    hit(s,2.1,.24)
    assert not rear_clear(s,(0.,0.,0.))


def test_unobserved_rear_and_insufficient_range_block_retreat():
    s=scan();hit(s,math.pi,float('nan'))
    assert not rear_clear(s,(0.,0.,0.))
    s=scan();s.ranges[:]=float('inf')
    assert rear_clear(s,(0.,0.,0.))
    s.range_max=.3
    assert not rear_clear(s,(0.,0.,0.))
    s=scan();s.ranges=s.ranges[160:480];s.angle_min=-math.pi/2
    assert not rear_clear(s,(0.,0.,0.))
