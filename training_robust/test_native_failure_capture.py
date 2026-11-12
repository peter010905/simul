from capture_native_failures import descendants,snapshot


def test_failure_capture_rejects_reused_pid_before_inspecting_other_processes(tmp_path):
    process=tmp_path/'123';process.mkdir()
    (process/'cmdline').write_text('/usr/bin/python3\0different_task.py\0')
    report=snapshot(123,tmp_path/'benchmark','heartbeat lost',proc=tmp_path)
    assert not report['verified_native_pid'] and 'processes' not in report


def test_process_family_does_not_capture_unrelated_runs():
    assert descendants(10,{10:1,11:10,12:11,20:1,21:20})=={10,11,12}
