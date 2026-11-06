import math

from osp_mission.native_audit import NativeAudit


def test_rotated_start_frame_and_false_ids(tmp_path):
    audit=NativeAudit({'targets':[{'id':42,'x':12.,'y':3.}]})
    audit.calibrate((10.,3.,math.pi/2),(0.,0.,0.))
    audit.observe(1.,11.,3.,0.)
    path=tmp_path/'markers.csv';path.write_text('id,x,y\n42,0,-2\n')
    result=audit.evaluate(path)
    assert result['correct_ids'] and result['correct_positions']
    assert result['approached_within_2m']==1
    path.write_text('id,x,y\n43,0,-2\n')
    assert not audit.evaluate(path)['correct_ids']
    path.write_text('id,x,y\n42,1,-2\n')
    assert not audit.evaluate(path)['correct_positions']


def test_missing_ground_truth_cannot_pass(tmp_path):
    audit=NativeAudit({'targets':[{'id':42,'x':1.,'y':2.}]})
    path=tmp_path/'markers.csv';path.write_text('id,x,y\n42,1,2\n')
    result=audit.evaluate(path)
    assert result['correct_ids']
    assert not result['available'] and not result['correct_positions']
