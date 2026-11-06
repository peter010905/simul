import math

import numpy as np

from osp_mission.persistent_edge_coverage import PersistentEdgeCoverage


def observed_wall():
    data = np.zeros((80, 80), dtype=np.int8)
    data[5:75, 40:44] = 100
    memory = PersistentEdgeCoverage()
    endpoints = [[2.005, y] for y in np.arange(1.5, 2.55, .05)]
    for stamp in (0., .2, .4):
        memory.observe(data, .05, (0., 0.), (1., 2., 0.), endpoints,
                       stamp, 665., 1.25)
    assert memory.viewed.any() and len(memory.anchors)
    return memory, data


def test_reassociates_small_raster_motion_without_crediting_other_side():
    memory, data = observed_wall()
    before = int(memory.viewed.sum())
    shifted = np.zeros_like(data)
    shifted[5:75, 41:45] = 100
    viewed = memory.grid(shifted, .05, (0., 0.))
    assert viewed[3, :, 41].sum() >= before
    assert not viewed[1, :, 44].any()
    assert not viewed[0].any() and not viewed[2].any()
    assert viewed.sum() <= len(memory.anchors)


def test_reprojection_never_generates_new_anchors_or_accumulates_drift():
    memory, data = observed_wall()
    anchors = memory.anchors.copy()
    for shift in (.02, .04, .06, .08, .10):
        memory.grid(data, .05, (shift, 0.))
    assert np.array_equal(memory.anchors, anchors)
    # Base grid retention must not let evidence creep with a shifted surface.
    assert not memory.viewed.any()


def test_removed_wall_and_far_new_wall_have_no_observation_credit():
    memory, data = observed_wall()
    empty = np.zeros_like(data)
    assert not memory.grid(empty, .05, (0., 0.)).any()
    empty[5:75, 50:54] = 100
    assert not memory.grid(empty, .05, (0., 0.)).any()


def test_only_three_real_frames_can_create_an_anchor():
    memory = PersistentEdgeCoverage()
    data = np.zeros((80, 80), dtype=np.int8)
    data[5:75, 40:44] = 100
    for stamp in (0., .2):
        memory.observe(data, .05, (0., 0.), (1., 2., 0.), [[2.005, 2.]],
                       stamp, 665., 1.25)
    assert not len(memory.anchors)
    memory.observe(data, .05, (0., 0.), (1., 2., math.pi), [[2.005, 2.]],
                   .4, 665., 1.25)
    assert not len(memory.anchors)


def test_diagonal_thin_wall_keeps_front_evidence_separate_from_back():
    data = np.zeros((90, 90), dtype=np.int8)
    y = np.arange(10, 80)
    data[y, 89-y] = 100
    points = np.column_stack(((89-y+.5)*.05, (y+.5)*.05))
    memory = PersistentEdgeCoverage()
    for stamp in (0., .2, .4):
        memory.observe(data, .05, (0., 0.), (1.2, 1.2, math.pi/4), points,
                       stamp, 665., 1.25)
    assert memory.viewed.any()
    before = memory.anchors.copy()
    memory.grid(data, .05, (.02, -.01))
    assert memory.viewed.any()
    assert not memory.viewed[1].any() and not memory.viewed[2].any()
    assert memory.viewed.sum() <= len(before)
    assert np.array_equal(before, memory.anchors)
