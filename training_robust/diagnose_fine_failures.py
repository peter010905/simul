"""Read-only evaluator diagnostics; hidden target data never enters the policy."""

from collections import Counter
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

from osp_mission.fast_sim import SearchEnv


class RecordedEnvironment(SearchEnv):
    def __init__(self, *args, **kwargs):
        self.camera_trace = []
        super().__init__(*args, **kwargs)

    def detect(self):
        self.camera_trace.append((self.x, self.y, self.yaw, self.elapsed))
        super().detect()


def run(reference):
    seed = reference['seed']
    cfg = yaml.safe_load(Path(f'scenarios_robust/v2/validation/{seed}.yaml').read_text())
    settings = dict(resolution=.05, lidar_samples=640, lidar_body_frame=True,
                    raycast_backend='native', continuous_shortcuts=True,
                    frame_quarter_turns=(seed // 20) % 4)
    env = RecordedEnvironment(cfg, seed=seed + 123456, planner='edge',
                              camera_width=960, camera_hz=5,
                              detection_dropout=cfg['sensor_stress']['detection_dropout'], **settings)
    env.speed = min(env.speed, cfg['sensor_stress']['speed_limit'])
    policy = json.loads(Path('training_robust/fidelity_validation/edge/policy.json').read_text())
    result = env.run(policy['parameters'])
    for key in result:
        if result[key] != reference[key]:
            raise AssertionError(f'Replay changed {seed} {key}: {result[key]} vs {reference[key]}')
    trace = np.asarray(env.camera_trace)
    missing = []
    for target in env.cfg['targets']:
        if target['id'] in env.found:
            continue
        dx, dy = target['x'] - trace[:, 0], target['y'] - trace[:, 1]
        distance = np.hypot(dx, dy)
        bearing = np.arctan2(dy, dx)
        relative = (bearing - trace[:, 2] + math.pi) % (2 * math.pi) - math.pi
        face = (-dx * math.cos(target['yaw']) - dy * math.sin(target['yaw'])) / np.maximum(distance, 1e-9)
        range_ok = (distance >= .25) & (distance <= .95 * env.camera_focal * target['size'] / 18)
        readable = range_ok & (face >= .45) & (distance <= .95 * env.camera_focal * target['size'] * face / 18)
        unoccluded = np.zeros(len(trace), dtype=bool)
        for i in np.flatnonzero(readable):
            ds = np.arange(.1, max(.11, distance[i] - .20), env.res * .35)
            xs = ((trace[i, 0] + ds * dx[i] / distance[i]) / env.res).astype(int).clip(0, env.map.shape[1] - 1)
            ys = ((trace[i, 1] + ds * dy[i] / distance[i]) / env.res).astype(int).clip(0, env.map.shape[0] - 1)
            unoccluded[i] = not env.truth[ys, xs].any()
        in_camera = unoccluded & (np.abs(relative) <= 1.25 / 2) & (distance * np.cos(relative) < 8.)
        if in_camera.any():
            cause = 'too_few_confirmed_frames'
        elif unoccluded.any():
            cause = 'camera_never_aimed_at_readable_front'
        elif readable.any():
            cause = 'readable_front_occluded'
        elif range_ok.any():
            cause = 'never_reached_readable_front'
        else:
            cause = 'never_within_pixel_range'
        missing.append(dict(id=target['id'], size=target['size'], mount=target['mount'],
                            confirmations=env.confirmations.get(target['id'], 0),
                            nearest=float(distance.min()), range_frames=int(range_ok.sum()),
                            readable_front_frames=int(readable.sum()),
                            unoccluded_front_frames=int(unoccluded.sum()),
                            usable_camera_frames=int(in_camera.sum()), cause=cause))
    out = Path('training_robust/fidelity_validation/failure_diagnostics')
    np.savez_compressed(out / f'{seed}.npz', trace=np.asarray(env.trace),
                        camera_trace=trace, observed=env.map, truth=env.truth, viewed=env.viewed)
    return dict(seed=seed, mount_profile=cfg['mount_profile'], obstacles=len(cfg['obstacles']),
                exact_replay=True, result=result, missing=missing)


def main():
    out = Path('training_robust/fidelity_validation/failure_diagnostics')
    out.mkdir(exist_ok=False)
    reference = json.loads(Path('training_robust/fidelity_validation/edge/completed.json').read_text())
    failures = [r for r in reference['rows'] if not r['success']]
    rows = []
    with ProcessPoolExecutor(max_workers=8) as pool:
        for row in pool.map(run, failures):
            rows.append(row)
            print(json.dumps(dict(completed=len(rows), total=len(failures))), flush=True)
    causes = dict(Counter(t['cause'] for r in rows for t in r['missing']))
    report = dict(split='v2/validation', planner='edge', test_loaded=False,
                  diagnostic_only_ground_truth=True, exact_replay=True,
                  source_hashes={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in Path('src/osp_mission/osp_mission').glob('*.py')},
                  missing_target_causes=causes, rows=rows)
    (out / 'completed.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(causes), flush=True)


if __name__ == '__main__':
    main()
