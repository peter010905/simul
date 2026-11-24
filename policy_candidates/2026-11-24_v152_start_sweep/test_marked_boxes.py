import numpy as np
import pytest
from scipy import ndimage
from osp_mission.marked_boxes import MarkedBoxes
from osp_mission._r32_navigation_core import Grid


def scene():
    cells = np.zeros((80, 80), dtype=int)
    for col in (25, 48):
        cells[30:42,col] = cells[30:42,col+11] = 100
        cells[30,col:col+12] = cells[41,col:col+12] = 100
    cells[5,:] = 100
    regions, _ = ndimage.label(cells >= 65, np.ones((3,3)))
    return Grid(cells,.05,(0.,0.,0.)), regions, ndimage.find_objects(regions)


def test_only_host_box_is_closed_and_registration_can_be_withdrawn():
    g, comp, bounds = scene()
    m=MarkedBoxes()
    targets={4:g.world(35,25), 7:g.world(5,30)}
    m.update(g,comp,bounds,comp>0,set(),targets)
    assert m.by_component == {comp[35,25]:{4}}
    assert comp[35,48] not in m.by_component
    m.update(g,comp,bounds,comp>0,set(),{})
    assert not m.markers and not m.by_component


def test_ambiguous_gap_between_two_surfaces_stays_open():
    g, comp, _=scene()
    comp[30:42,38]=7
    m=MarkedBoxes()
    m.update(g,comp,ndimage.find_objects(comp),comp>0,set(),{2:g.world(35,37)})
    assert not m.by_component
