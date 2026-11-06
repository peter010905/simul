"""Check the optimized sampler against the frozen screening implementation."""

import importlib.util
import json
from pathlib import Path
import time

import numpy as np
import yaml

from evaluate import task
from osp_mission.plane_views import symmetric_grid


def main():
    path = Path('training_robust/geometry_view_screen/source_snapshot/plane_views.py')
    spec = importlib.util.spec_from_file_location('frozen_plane_views', path)
    frozen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(frozen)
    rng = np.random.default_rng(760163)
    timings = {'frozen': 0., 'optimized': 0.}
    for case in range(120):
        shape = tuple(rng.integers(20, 180, size=2))
        grid = np.zeros(shape, dtype=np.int8)
        grid[rng.random(shape) < .06] = 100
        grid[:rng.integers(0, shape[0] // 3), :] = -1
        safe = (grid == 0) & (rng.random(shape) > .04)
        distances = rng.uniform(0., 30., shape)
        distances[rng.random(shape) < .01] = np.inf
        resolution = [.05, .1, .2, .125][case % 4]
        args = (grid, resolution, (0., 0.), (1., 1.), safe, distances, None, None)
        start = time.perf_counter(); before = frozen.symmetric_grid(*args)
        timings['frozen'] += time.perf_counter() - start
        start = time.perf_counter(); after = symmetric_grid(*args)
        timings['optimized'] += time.perf_counter() - start
        assert before == after, (case, shape, resolution, len(before), len(after))
    rows = json.loads(Path('training_robust/geometry_view_screen/symmetric_edge.json').read_text())['rows']
    weights = json.loads(Path('src/osp_mission/config/edge_policy.json').read_text())['parameters']
    replays = []
    for seed in [8500107, 8500139]:
        cfg = yaml.safe_load(Path(f'scenarios_robust/v2/train/{seed}.yaml').read_text())
        settings = dict(resolution=.05, lidar_samples=640, lidar_body_frame=True,
                        raycast_backend='native', continuous_shortcuts=True,
                        frame_quarter_turns=(seed // 20) % 4)
        actual = task((cfg, 'symmetric_edge', weights, 5., 3, settings))
        expected = next(row for row in rows if row['seed'] == seed)
        assert actual == expected, (seed, actual, expected)
        replays.append(dict(seed=seed, exact_metrics_match=True))
        print(json.dumps(replays[-1]), flush=True)
    result = dict(randomized_cases=120, exact_candidate_match=True,
                  function_wall_seconds=timings, replays=replays,
                  note='Timing is computation, not simulated mission speed')
    Path('training_robust/symmetric_sampler_equivalence.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
