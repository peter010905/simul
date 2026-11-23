"""Export matched native simulation results without scoring unfinished or incomparable trials."""
import argparse
import csv
import json
from pathlib import Path
import sys


def read(path):
    return json.loads(path.read_text())


def result(directory):
    manifest = read(directory / 'protocol.json')
    score = read(directory / 'exam-score.json')
    camera = read(directory / 'camera_metrics.json')
    name = directory.name.split('-')[0].removeprefix('run')
    behavior_file = directory / 'behavior.json'
    behavior = read(behavior_file) if behavior_file.is_file() else {}
    observed = behavior.get('sensors', {})
    coverage = observed.get('surface_coverage', {}) if observed.get('valid') else {}
    if (manifest['camera'] != [640, 480, 5] or manifest['physical_drive'] or
            manifest['duration_sim_s'] != 600 or manifest['furniture'] != 'fixed' or
            manifest['public_marker_count'] != 3 or
            (camera['width'], camera['height'], camera['requested_fps']) != (640, 480, 5)):
        raise ValueError(f'Incompatible trial protocol: {directory}')
    return dict(version=name, seed=manifest['seed'], valid=score['valid'], strict=score['strict'],
                verified=score['verified'], wall=score['wall_near'], box=score['obstacle_near'],
                completion_s=score['goal_complete_by_s'], elapsed_s=round(score['elapsed_s'], 2),
                path_m=round(score['travel_m'], 2), bumps=score['bumps'], false_ids=len(score['false_ids']),
                map_error_max_m=score['max_saved_error_m'], stop_reason=score['stop_reason'],
                goals=behavior.get('dispatches'), nav_cancels=observed.get('navigate', {}).get('canceled'),
                nav_aborts=observed.get('navigate', {}).get('aborted'), turns_at_rest_s=behavior.get('turns_at_rest_s'),
                stationary_s=behavior.get('almost_still_s'),
                repeat_no_new_m=behavior.get('route', {}).get('repeated_no_new_view_m'),
                wall_seen_fraction=coverage.get('wall', {}).get('fraction'),
                object_seen_fraction=coverage.get('obstacle', {}).get('fraction'),
                boxes_all_four=coverage.get('boxes_all_four'), boxes=coverage.get('boxes'),
                processed_frames=camera['processed_frames'], input_frames=camera['input_frames'],
                image_p95_s=camera['processed_age_sim_p95_s'], fixture=manifest['fixture_sha256'],
                effective_world=manifest['effective_world_sha256'])


def collect(directory):
    rows = [result(path.parent) for path in sorted(directory.glob('run*-*/complete.json'))]
    fixtures = {}
    for row in rows:
        world = (row['fixture'], row['effective_world'])
        if fixtures.setdefault(row['seed'], world) != world:
            raise ValueError(f"Seed {row['seed']} has different exam worlds across trials")
    return sorted(rows, key=lambda r: (r['seed'], int(r['version'])))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path, help='archive containing run<version>-<seed> folders')
    args = parser.parse_args()
    rows = collect(args.directory)
    if rows:
        writer = csv.DictWriter(sys.stdout, list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
