"""Stage sensor-confirmed approach completion with a physical stopping margin."""
import argparse
from pathlib import Path
import shutil

from prepare_approach_witness_runtime import prepare as add_witnesses, replace_once

ROOT = Path(__file__).resolve().parents[1]


def prepare(source, destination):
    add_witnesses(source, destination)
    package = destination / 'src/osp_mission'
    path = package / 'osp_mission/marker_approach.py'
    code = path.read_text()
    code = replace_once(code, 'near_distance<=1.40:', 'near_distance<=self.radius:')
    code = replace_once(code, 'conservative_radius_m=1.40,', 'conservative_radius_m=self.radius,')
    for field in ('odom_point', 'camera_point', 'lidar_camera_point'):
        code = replace_once(code, f"evidence.get('{field}',[])", f"(evidence.get('{field}') or [])")
    start = code.index('    def verified(self, ident, target=None):')
    end = code.index('    def reconcile_targets(', start)
    code = code[:start] + '''    def verified(self, ident, target=None):
        # Map-pose proximity remains diagnostic: only sensor-relative frames
        # can grant completion. Keep that proof through later SLAM corrections.
        return ident in self.confirmed_ids and ident in self.range_witnesses

''' + code[end:]
    code = replace_once(code, '''        """Credit a fresh localization pose after the ID has been confirmed.

        Camera confirmation remains the detector's responsibility. An approach
        does not require a second simultaneous decoding of that same ID.
        """''', '''        """Record fresh map-pose proximity for diagnostics, never completion."""''')
    code = replace_once(code, "verification_mode='confirmed_id_and_independent_fresh_localization',",
        "verification_mode='confirmed_id_and_three_readable_sensor_relative_frames',\n"
        "                    sensor_proof_radius_m=self.radius, required_sensor_frames=3,")
    code = replace_once(code, 'for standoff in (.9, 1.05, 1.2):',
        '# .90 m + Nav2 .25 m stopping tolerance leaves room for the\n'
        '                # 1.30 m sensor proof, inside the actual 1.50 m task limit.\n'
        '                for standoff in (.75, .85, .9):')
    code = replace_once(code, '(self.radius if c.get(\'stationary\') else 1.2)',
        '(self.radius if c.get(\'stationary\') else .9)')
    path.write_text(code)
    # Replace obsolete map-only completion assertions, retaining the original
    # evaluator radius and collision/line-of-sight checks in the new tests.
    (package / 'test/test_approach_witness_runtime.py').unlink()
    shutil.copy2(ROOT / 'scripts/test_strict_approach_runtime.py', package / 'test/test_marker_approach.py')
    return destination


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--destination', type=Path, required=True)
    args = parser.parse_args()
    print(prepare(args.source.resolve(), args.destination.resolve()))
