"""Compare acquisition fidelity, never present a sensor probe as mission success."""
import argparse
import json
from pathlib import Path


def compare(baseline, measured, profile):
    b, m = [json.loads((p/'sensors.json').read_text()) for p in (baseline, measured)]
    manifests = [json.loads((p/'manifest.json').read_text()) for p in (baseline, measured)]
    if (not b['complete'] or not m['complete'] or
            any(x['physical_drive'] or not x['no_mission'] for x in manifests) or
            manifests[0]['fixture_sha256'] != manifests[1]['fixture_sha256']):
        raise ValueError('Need two completed, identical-world sensor-only probes')
    target = json.loads(profile.read_text())
    def row(d):
        return dict(camera_dimensions=d['shapes']['image'],
            camera_hz=d['topics']['/oakd/rgb/preview/image_raw']['median_hz'],
            scan_hz=d['topics']['/scan']['median_hz'], **d['shapes']['lidar'])
    actual = row(m)
    checks = dict(camera_dimensions=actual['camera_dimensions'] == [target['camera']['width'], target['camera']['height']],
        camera_rate=abs(actual['camera_hz']/target['camera']['fps']-1) < .02,
        lidar_rate=abs(actual['scan_hz']/target['lidar']['hz']-1) < .02,
        lidar_beams=actual['beams'] == target['lidar']['beams'],
        lidar_raw_range=abs(actual['raw_max_range_m']-target['lidar']['raw_max_range_m']) < .01)
    return dict(evidence_type='matched_native_exam_sensor_probe_not_mission', fixture_sha256=manifests[0]['fixture_sha256'],
        baseline=row(b), measured=actual, checks=checks, acquisition_match_passed=all(checks.values()),
        actual_slam_range_in_probe_tested=False, navigation_or_detection_improvement_tested=False,
        physical_drive=False, unknowns=['Optical distortion/FOV calibration', 'Rolling scan timing and missing returns',
            'Wheel slip, payload motion and real contact geometry', 'Recorded sensor interruptions under mission load'])


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('baseline', type=Path)
    p.add_argument('measured', type=Path)
    p.add_argument('profile', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    result = compare(a.baseline, a.measured, a.profile)
    a.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    if not result['acquisition_match_passed']:
        raise SystemExit(1)
