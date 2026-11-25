"""Measured acquisition profile only; never fit friction/noise from wheel odometry."""
import json
from pathlib import Path
import statistics


def build(rows, slam_range, heldout):
    if slam_range not in (5., 12.):
        raise ValueError('Use an explicitly separated 5m or 12m SLAM profile')
    training = [r for r in rows if r['run'] not in heldout]
    validation = [r for r in rows if r['run'] in heldout]
    if not training or len(validation) != len(heldout):
        raise ValueError('Require disjoint, whole-run calibration and validation')
    # Acquisition is common across the two SLAM ranges; do not fit occupancy or
    # missing-return probability by pooling scene-dependent finite-ray ratios.
    cameras = [r['cameras']['/mission/processed_camera'][0] for r in training]
    shapes = {(c['width'], c['height']) for c in cameras}
    if len(shapes) != 1:
        raise ValueError('Different actual detector resolutions require separate profiles')
    geometries = [r['scans']['/scan']['geometries'][0] for r in training]
    if len({(g['beams'], g['angle_min'], g['angle_max']) for g in geometries}) != 1:
        raise ValueError('Mixed LiDAR geometry')
    width, height = shapes.pop()
    scan_hz = round(statistics.median(r['timing']['/scan']['median_hz'] for r in training), 2)
    camera_hz = round(statistics.median(r['timing']['/mission/processed_camera']['median_hz'] for r in training), 2)
    raw_ranges = {float(k) for r in training for k in r['scans']['/scan']['range_max_counts']}
    if len(raw_ranges) != 1:
        raise ValueError('Mixed raw LiDAR max ranges')
    raw_max = raw_ranges.pop()
    if slam_range > raw_max:
        raise ValueError('SLAM range cannot exceed measured acquisition range')
    return dict(schema_version=1, physical_drive=False, slam_max_range_m=slam_range,
        camera=dict(width=width, height=height, fps=camera_hz,
            calibration='Resolution/rate only. Optical model and distortion require independent calibration.'),
        lidar=dict(hz=scan_hz, beams=geometries[0]['beams'], angle_min=geometries[0]['angle_min'],
            angle_max=geometries[0]['angle_max'], raw_max_range_m=raw_max),
        calibration_runs=[r['run'] for r in training], validation_runs=list(heldout),
        heldout_errors=[dict(run=r['run'], scan_rate_error_percent=100*abs(scan_hz/r['timing']['/scan']['median_hz']-1),
            camera_rate_error_percent=100*abs(camera_hz/r['timing']['/mission/processed_camera']['median_hz']-1),
            detector_dimensions_match=(r['cameras']['/mission/processed_camera'][0]['width'],
                                       r['cameras']['/mission/processed_camera'][0]['height']) == (width, height))
            for r in validation],
        limitations=['Scene-dependent no-return rates are not simulated as random dropped rays',
            'Recorded camera age includes recorder/processing paths and is not injected as pure transport delay',
            'No independently measured payload friction, slip, motor lag or LiDAR range noise fit',
            'Native 640x480 5fps remains a separate intended-camera sensitivity profile',
            'Simulation sensor matching is not evidence of improved physical mission success'])


def apply(doc, profile):
    if profile.get('physical_drive') is not False:
        raise ValueError('Only simulation profiles are accepted')
    import math
    c, l = profile['camera'], profile['lidar']
    if (any(not isinstance(c[k], int) or not 32 <= c[k] <= 1920 for k in ('width', 'height'))
            or not math.isfinite(c['fps']) or not 1 <= c['fps'] <= 60
            or not math.isfinite(l['hz']) or not 1 <= l['hz'] <= 40
            or not isinstance(l['beams'], int) or not 16 <= l['beams'] <= 4096
            or not -math.pi <= l['angle_min'] < l['angle_max'] <= math.pi+.00001
            or not math.isfinite(l['raw_max_range_m']) or not .5 <= l['raw_max_range_m'] <= 50):
        raise ValueError('Invalid measured acquisition geometry')
    def set_text(parent, tag, value):
        nodes = parent.getElementsByTagName(tag)
        if not nodes:
            raise ValueError('Missing required robot sensor tag: '+tag)
        for node in nodes:
            while node.firstChild:
                node.removeChild(node.firstChild)
            node.appendChild(doc.createTextNode(str(value)))
    cameras = lidars = 0
    for sensor in doc.getElementsByTagName('sensor'):
        kind = sensor.getAttribute('type')
        if kind in ('rgbd_camera',):
            cameras += 1
            set_text(sensor, 'update_rate', profile['camera']['fps'])
            for image in sensor.getElementsByTagName('image'):
                set_text(image, 'width', profile['camera']['width'])
                set_text(image, 'height', profile['camera']['height'])
        elif kind in ('gpu_ray', 'gpu_lidar', 'ray', 'lidar') and sensor.getAttribute('name') == 'rplidar':
            lidars += 1
            set_text(sensor, 'update_rate', profile['lidar']['hz'])
            for horizontal in sensor.getElementsByTagName('horizontal'):
                set_text(horizontal, 'samples', profile['lidar']['beams'])
                set_text(horizontal, 'min_angle', profile['lidar']['angle_min'])
                set_text(horizontal, 'max_angle', profile['lidar']['angle_max'])
            for ranges in sensor.getElementsByTagName('range'):
                set_text(ranges, 'max', profile['lidar']['raw_max_range_m'])
    if cameras != 1 or lidars != 1:
        raise ValueError(f'Expected one RGBD camera and one LiDAR, found {cameras}, {lidars}')
    return doc


if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('audit', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    rows = json.loads(a.audit.read_text())
    for limit in (5., 12.):
        result = build(rows, limit, ['20261125T174857-2f02f6e167', '20261125T195426-a84b4e02f7'])
        a.output.mkdir(parents=True, exist_ok=True)
        (a.output/f'measured-slam{limit:g}.json').write_text(json.dumps(result, indent=2)+'\n')
