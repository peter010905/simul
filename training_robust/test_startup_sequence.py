import json

import pytest

from audit_startup_sequence import audit


@pytest.mark.parametrize('times,expected',[([100.3,101.],True),([99.,100.3],False),([],False)])
def test_native_readiness_requires_actual_controller_logs_and_no_earlier_start(tmp_path,times,expected):
    (tmp_path/'startup_guard.json').write_text(json.dumps(dict(success=True,observed_slam_state=3,
        map_ready=True,reason='slam_active_map_tf_scan_ready',finished_wall_epoch_s=100.)))
    (tmp_path/'launch.log').write_text('\n'.join(
        f'[controller_server-42] [INFO] [{stamp:.9f}] [controller_server]: Creating controller server'
        for stamp in times))
    assert audit(tmp_path)['nav2_started_after_readiness_verified'] is expected
