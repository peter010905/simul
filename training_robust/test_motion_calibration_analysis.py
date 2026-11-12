import csv

import pytest

from analyze_motion_calibration import analyze,extract


def fixture(path,wheel_ratio,ekf_ratio,gap=False):
    with path.open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['stream','stamp_sim_s','x','y','yaw','linear_x'])
        for i in range(701):
            stamp=i/50
            if not gap or not 4<stamp<6:writer.writerow(['ground_truth',stamp,.3*stamp,0,0,''])
            writer.writerow(['wheel',stamp,'','','',.3*wheel_ratio])
            writer.writerow(['ekf',stamp,'','','',.3*ekf_ratio])


@pytest.mark.parametrize('wheel,ekf,scale',[(1.2,1.,.83),(1.,1.2,1.)])
def test_encoder_and_filter_bias_are_reported_separately(tmp_path,wheel,ekf,scale):
    train=tmp_path/'train.csv';valid=tmp_path/'valid.csv';fixture(train,wheel,ekf);fixture(valid,wheel,ekf)
    report=analyze(train,valid)
    assert report['rounded_candidate_scale']==pytest.approx(scale)
    assert report['validation']['median_raw_to_actual_ratio']==pytest.approx(wheel)
    assert report['validation']['median_ekf_to_actual_ratio']==pytest.approx(ekf)


def test_unsampled_motion_cannot_be_used_as_a_calibration_window(tmp_path):
    path=tmp_path/'gap.csv';fixture(path,1.2,1.2,gap=True)
    report=extract(path)
    assert report['rejected']['sampling_gap']>=2
    assert all(not 4<=row['start_sim_s']<=5 for row in report['windows'])
