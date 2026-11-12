from types import SimpleNamespace

import numpy as np
import pytest

from record_depth_patch_quality import patch_quality


@pytest.mark.parametrize('encoding,dtype,value', [('16UC1','<u2',1200),('32FC1','>f4',1.2)])
def test_metric_patch_handles_depth_units_endianness_and_padded_rows(encoding,dtype,value):
    array=np.zeros((7,9),dtype=dtype);array[:,:7]=value
    array[2,2]=0
    message=SimpleNamespace(encoding=encoding,is_bigendian=dtype.startswith('>'),
        width=7,height=7,step=9*array.dtype.itemsize,data=array.tobytes())
    report=patch_quality(message,[3.,3.])
    assert report['patch_values']==25 and report['valid_values']==24
    assert report['median_m']==pytest.approx(1.2)
    assert report['median_absolute_deviation_m']==0


def test_invalid_buffer_or_outside_center_cannot_be_reported_as_good_depth():
    message=SimpleNamespace(encoding='16UC1',is_bigendian=False,width=7,height=7,step=14,data=b'')
    assert not patch_quality(message,[3,3])['available']
    message.data=np.ones((7,7),dtype='<u2').tobytes()
    assert not patch_quality(message,[8,3])['available']
    report=patch_quality(message,[3,3])
    assert report['valid_values']==0 and 'median_m' not in report
