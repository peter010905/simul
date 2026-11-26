"""A face swept from its standoff is not re-chosen during the same box visit."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / 'overlay'))

from osp_mission.box_continuity import BoxContinuity  # noqa: E402


def face(x, y, normal=0.0, box=(0.0, 0.0)):
    return dict(r32_kind='interior_surface', face_inspection=True, surface_odom=[x, y],
                normal_odom=normal, batch_box=list(box), previous_attempts=0)


def test_swept_face_is_not_repeated():
    c = BoxContinuity()
    a, b = face(.4, 0.), face(0., .4, math.pi / 2)
    c.dispatched(a, 10.)
    c.begin_sweep(1.0, 11.)
    c.result('spin', 4, '', 13.)
    assert c.recently_swept(dict(a, surface_odom=[.43, .03]), 14.)
    assert not c.recently_swept(b, 14.)
    assert not c.recently_swept(a, 13. + 55. + 90. + 1)  # hold counts from the sweep
    assert [g for g in (a, b) if not c.recently_swept(g, 14.)] == [b]


if __name__ == '__main__':
    test_swept_face_is_not_repeated()
    print('ok')
