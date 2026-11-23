import math

import numpy as np

from osp_mission.edge_policy import inside_coverage_range, readable_distance
from osp_mission.visual_survey import VisualSurvey


def test_camera_evidence_unchanged_at_boundaries():
    distances = np.linspace(.05, 4., 200)
    cosines = np.linspace(.30, 1., 91)[:, None]
    for fx in (173., 334., 443.53):
        limit = readable_distance(fx)
        expected = (distances > .25) & (cosines > .6) & (distances < limit * cosines)
        np.testing.assert_array_equal(inside_coverage_range(distances, cosines, limit), expected)
        assert not inside_coverage_range(limit, 1., limit)
        assert not inside_coverage_range(.25, 1., limit)


def test_wall_inspection_can_earn_credit_but_cues_keep_far_options():
    survey = VisualSurvey()
    survey.camera = (443.53, math.radians(70.))
    options = survey.standoffs()
    frontal = survey.coverage_limit()
    assert 1.67 < frontal < 1.68
    assert inside_coverage_range(options[-1], 1., frontal)
    assert max(options) > 2.9
    assert not inside_coverage_range(max(options), 1., frontal)
    assert not inside_coverage_range(options[-1], .65, frontal)
    assert len(options) == len(set(options))
