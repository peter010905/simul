"""Keep three individually valid near-range sensor frames within one second."""
import argparse
from pathlib import Path
import shutil

from prepare_approach_witness_runtime import replace_once

ROOT = Path(__file__).resolve().parents[1]


def prepare(source, destination):
    if destination.exists(): raise FileExistsError('Use a new immutable candidate')
    package = destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission', package,
        ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache'))
    path = package/'osp_mission/marker_approach.py'; code = path.read_text()
    code = replace_once(code, '    radius = 1.3', '    radius = 1.3\n    witness_window_s = 1.0')
    old = '''        if valid and near_distance<=self.radius:
            rows=self.witness_runs.setdefault(ident,[])
            if rows and (stamp-rows[-1]['stamp']>.6
                         or np.linalg.norm(odom_point-np.asarray(rows[-1]['point_odom']))>.25):
                rows.clear()'''
    new = '''        rows=self.witness_runs.setdefault(ident,[])
        rows[:]=[row for row in rows if 0.<=stamp-row['stamp']<=self.witness_window_s+1e-9]
        if valid and near_distance<=self.radius:
            if rows and np.linalg.norm(odom_point-np.asarray(rows[-1]['point_odom']))>.25:
                rows.clear()'''
    code = replace_once(code, old, new)
    code = replace_once(code, 'conservative_radius_m=self.radius,observations=[dict(r) for r in rows])',
        'conservative_radius_m=self.radius,time_window_sim_s=self.witness_window_s,\n'
        '                    observations=[dict(r) for r in rows])')
    code = replace_once(code, '''        else:
            self.witness_runs.pop(ident,None)''', '''        elif valid or not confirmed:
            # An explicit far measurement or unconfirmed ID breaks the run.
            self.witness_runs.pop(ident,None)
        # Missing/conflicting auxiliary measurements add no evidence. Earlier
        # individually validated frames expire after1s instead of being erased.
''')
    code = replace_once(code, 'sensor_proof_radius_m=self.radius, required_sensor_frames=3,',
        'sensor_proof_radius_m=self.radius, required_sensor_frames=3,\n'
        '                    witness_window_sim_s=self.witness_window_s,')
    path.write_text(code)
    path = package/'test/test_marker_approach.py'; tests = path.read_text()
    tests = replace_once(tests, 'def test_duplicates_gaps_and_intermittent_invalid_frames_reset_accumulation():',
        'def test_duplicates_and_old_frames_do_not_count_but_missing_samples_preserve_recent_valid_frames():')
    tests = replace_once(tests, '''    for stamp in (3.6, 3.8): observe(tracker, stamp)
    assert not tracker.verified(44)
    observe(tracker, 4.)
    assert tracker.verified(44)''', '''    observe(tracker, 3.6)
    assert tracker.verified(44)
    assert [r['stamp'] for r in tracker.range_witnesses[44]['observations']] == [3., 3.2, 3.6]''')
    path.write_text(tests)
    shutil.copy2(ROOT/'scripts/test_windowed_witness_runtime.py', package/'test/test_windowed_witness_runtime.py')
    return destination


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('source', 'destination'): parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args(); print(prepare(args.source.resolve(), args.destination.resolve()))
