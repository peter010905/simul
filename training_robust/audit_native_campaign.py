"""Read-only verification of native outcomes, files and recorded video.

Uses evaluator ground truth after a run, never as an input to the robot.
The 90% gate is reported only for a complete 40-scene campaign.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


def audit_episode(directory, entry):
    directory = Path(directory)
    result = json.loads((directory / 'evaluation.json').read_text())
    errors = []
    def require(condition, message):
        if not condition:
            errors.append(message)
    scene = Path(entry['file'])
    require(hashlib.sha256(scene.read_bytes()).hexdigest() == entry['sha256'], 'Scenario hash changed')
    cfg = yaml.safe_load(scene.read_text())
    closest = {str(t['id']): math.inf for t in cfg['targets']}
    trace = directory / 'ground_truth_trajectory.csv'
    if trace.exists():
        with trace.open() as stream:
            for row in csv.DictReader(stream):
                x, y = float(row['world_x']), float(row['world_y'])
                if not math.isfinite(x) or not math.isfinite(y):
                    continue
                for target in cfg['targets']:
                    ident = str(target['id'])
                    closest[ident] = min(closest[ident], math.hypot(x-target['x'], y-target['y']))
    require(len(closest) == 3 and all(d <= 1.5 for d in closest.values()),
            'Actual trajectory must approach every marker within 1.5 m')
    outputs = list(directory.glob('*/result.json'))
    require(len(outputs) == 1, 'Exactly one mission result directory is required')
    coordinate_errors = {}
    if len(outputs) == 1:
        saved = outputs[0].parent
        for name in ['map.pgm', 'map.yaml', 'map.posegraph', 'map.data', 'markers.csv']:
            file = saved / name
            require(file.is_file() and file.stat().st_size > 0, f'Missing/empty {name}')
        grid = cv2.imread(str(saved / 'map.pgm'), cv2.IMREAD_GRAYSCALE)
        require(grid is not None and grid.size > 100, 'Map image cannot be decoded')
        if (saved / 'map.yaml').exists():
            metadata = yaml.safe_load((saved / 'map.yaml').read_text())
            require(metadata['resolution'] > 0 and len(metadata['origin']) == 3, 'Invalid map geometry')
            require((saved / metadata['image']).resolve() == (saved / 'map.pgm').resolve(), 'Map YAML points elsewhere')
        if (saved / 'markers.csv').exists():
            with (saved / 'markers.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            estimates = {int(r['id']): np.array([float(r['x']), float(r['y'])]) for r in rows}
            targets = {int(t['id']): t for t in cfg['targets']}
            require(len(rows) == len(estimates) == 3 and set(estimates) == set(targets), 'Three distinct correct marker IDs are required')
            alignment = result.get('ground_truth_audit', {}).get('world_to_map')
            require(alignment is not None, 'Initial frame alignment is missing')
            if alignment is not None:
                ox, oy, angle = alignment
                c, s = math.cos(angle), math.sin(angle)
                for ident in set(estimates) & set(targets):
                    target = targets[ident]
                    expected = np.array([ox+c*target['x']-s*target['y'], oy+s*target['x']+c*target['y']])
                    distance = float(np.linalg.norm(estimates[ident]-expected))
                    coordinate_errors[str(ident)] = distance
                    require(math.isfinite(distance) and distance <= .5, f'Marker {ident} coordinate error exceeds 0.5 m')
    require(result.get('reason') == 'all_targets_found', 'Mission did not find all targets')
    require(0 < result.get('elapsed_seconds', math.inf) <= 600, 'Mission exceeded 600 seconds')
    require(result.get('collisions', -1) == 0, 'Recorded collisions are nonzero or unavailable')
    video = directory / 'video/mission.mp4'
    capture = cv2.VideoCapture(str(video))
    frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = capture.get(cv2.CAP_PROP_FPS)
    first, _ = capture.read()
    capture.set(cv2.CAP_PROP_POS_FRAMES, max(0, frames-1))
    last, _ = capture.read()
    capture.release()
    recording = result.get('recording', {})
    video_valid = bool(first and last and fps > 0 and frames == recording.get('frames')
                       and recording.get('complete_visual_sources') and not recording.get('error'))
    require(video_valid, 'Actual video sources or first/last frame decoding failed')
    if video_valid:
        require(abs(frames/fps-result.get('elapsed_seconds', 0)) <= 1.5, 'Video duration differs from the mission')
    return dict(seed=entry['seed'], success=not errors, evaluator_success=result.get('success'),
                seconds=result.get('elapsed_seconds'), targets=result.get('targets'),
                collisions=result.get('collisions'), coordinate_errors_m=coordinate_errors,
                required_approach_m=1.5,
                closest_approach_m={i:d if math.isfinite(d) else None for i,d in closest.items()},
                video=dict(file=str(video.resolve()), frames=frames, fps=fps, verified=video_valid),
                errors=errors)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('campaign')
    args = parser.parse_args()
    root = Path(args.campaign)
    manifest = json.loads((root / 'manifest.json').read_text())
    report = json.loads((root / 'report.json').read_text())
    # report.json is updated only after the complete evaluator/recorder cleanup.
    rows = [audit_episode(root / 'episodes' / str(r['scenario']['seed']), r['scenario'])
            for r in report['rows']]
    seeds = [r['seed'] for r in rows]
    planned = len(manifest['scenarios'])
    complete = len(rows) == planned and len(set(seeds)) == planned
    successes = sum(r['success'] for r in rows)
    audit = dict(planned=planned, audited=len(rows), complete=complete, successes=successes,
                 observed_success_rate=successes/len(rows) if rows else None,
                 native_40_scene_90_percent_gate=complete and planned == 40 and successes >= 36,
                 note='Development campaigns do not establish held-out performance. Gate alone does not verify split independence.',
                 rows=rows)
    (root / 'artifact_audit.json').write_text(json.dumps(audit, indent=2)+'\n')
    print(json.dumps({k:v for k,v in audit.items() if k != 'rows'}, indent=2))


if __name__ == '__main__':
    main()
