import numpy as np
from osp_mission import visual_cue


def test_two_matching_frames_with_a_missed_frame_and_cooldown(monkeypatch):
    monkeypatch.setattr(visual_cue, 'paper_candidate', lambda image, quad: quad)
    monkeypatch.setattr(visual_cue.cv2.aruco, 'detectMarkers', lambda *a:([],None,[]))
    v = visual_cue.VisualCue()
    def step(t, x=80, readable=False):
        return v.update(None, [(x,40,16)], readable, t, 150, 100)
    assert step(1) is None and step(1) is None
    assert v.update(None, [], False, 1.1, 150, 100) is None
    out = step(1.2)
    assert out['angle'] > 0 and 'id' not in out
    assert step(2) is None
    assert step(20, readable=True) is None and step(20.1) is None
    assert step(23) is None and step(23.1,10) is None and step(23.2) is None


def test_plain_sign_and_window_are_not_inspection_cues():
    im = np.full((100,100), 220, np.uint8)
    quad = np.float32([[[20,20],[70,20],[70,70],[20,70]]])
    im[20:71,20:71] = 20
    assert visual_cue.paper_candidate(im,quad) is None
    im[28:63,28:63] = 220
    assert visual_cue.paper_candidate(im,quad) is None
