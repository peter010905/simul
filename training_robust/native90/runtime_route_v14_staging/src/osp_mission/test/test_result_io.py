from osp_mission.result_io import read_result, write_result


def test_partial_result_waits_then_complete_result_is_read(tmp_path):
    path=tmp_path/'result.json'
    assert read_result(path) is None
    path.write_text('')
    assert read_result(path) is None
    path.write_text('{"reason":')
    assert read_result(path) is None
    value={'reason':'all_targets_found','targets':3}
    write_result(path,value)
    assert read_result(path)==value
    assert not (tmp_path/'result.json.tmp').exists()


def test_staging_file_never_appears_as_completed_result(tmp_path):
    (tmp_path/'result.json.tmp').write_text('{"reason":')
    assert not list(tmp_path.glob('result.json'))
    assert read_result(tmp_path/'result.json') is None
