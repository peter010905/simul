"""Prepare a bounded parameter search evaluated by Gazebo, SLAM and Nav2."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / 'training_robust'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    source = HERE / 'native90/runtime_fix_v13_observation_staging/src/osp_mission'
    candidates = {}
    for name in ('control', 'stable_map', 'frontier_balanced'):
        workspace = output / 'workspaces' / name
        subprocess.run([sys.executable, str(ROOT / 'scripts/prepare_robot_runtime.py'),
                        '--source', str(source), '--destination', str(workspace)], check=True)
        package = workspace / 'src/osp_mission'
        edge_path = package / 'osp_mission/edge_policy.py'
        code = edge_path.read_text()
        old = '.85*focal*.12*incidence/18'
        if code.count(old) != 1:
            raise ValueError('Review the marker-size prior before preparing another stage.')
        edge_path.write_text(code.replace(old, '.85*focal*.08*incidence/18'))
        # Reproduce the earlier false coverage credit, now requiring zero credit.
        (package / 'test/test_small_marker_coverage.py').write_text('''import numpy as np
from osp_mission.edge_policy import EdgeCoverage


def test_eight_cm_marker_below_minimum_pixels_is_not_credited():
    focal = 960 / (2 * np.tan(1.25 / 2))
    grid = np.zeros((50, 80), dtype=np.int8)
    grid[:, 50] = 100
    coverage = EdgeCoverage()
    for stamp in (1., 1.2, 1.4):
        coverage.observe(grid, .1, (0., 0.), (1.5, 2.05, 0.),
                         np.array([[5.02, 2.05]]), stamp, focal, 1.25)
    assert coverage.viewed.sum() == 0
''')
        nav_path = package / 'config/nav2_gazebo.yaml'
        slam_path = package / 'config/slam.yaml'
        nav = yaml.safe_load(nav_path.read_text())
        slam = yaml.safe_load(slam_path.read_text())
        policy_path = package / 'config/edge_policy.json'
        policy = json.loads(policy_path.read_text())
        changes = ['ROS time for sensor evidence', '8cm conservative coverage prior']
        if name != 'control':
            slam['slam_toolbox']['ros__parameters'].update(
                map_update_interval=.5, minimum_travel_distance=.05,
                minimum_travel_heading=.05)
            controller = nav['controller_server']['ros__parameters']
            controller['progress_checker'].update(required_movement_radius=.25,
                                                  movement_time_allowance=12.)
            speed = .24 if name == 'stable_map' else .30
            angular = .65 if name == 'stable_map' else .75
            controller['FollowPath'].update(desired_linear_vel=speed,
                                            rotate_to_heading_angular_vel=angular)
            nav['velocity_smoother']['ros__parameters'].update(
                max_velocity=[speed, 0., angular], min_velocity=[-speed, 0., -angular])
            changes += ['0.5s map update', '5cm/0.05rad SLAM motion thresholds',
                        '0.25m/12s progress checker', f'RPP {speed}m/s, {angular}rad/s']
        if name == 'frontier_balanced':
            policy['parameters'][1] *= 2.
            policy['parameters'][2] *= 1.25
            changes += ['2x frontier gain', '1.25x travel cost']
        policy.update(status='slam_nav2_native_search_candidate', changes=changes)
        policy_path.write_text(json.dumps(policy, indent=2) + '\n')
        nav_path.write_text(yaml.safe_dump(nav))
        (package / 'config/nav2_robot_policy.yaml').write_text(yaml.safe_dump(nav))
        slam_path.write_text(yaml.safe_dump(slam))
        candidates[name] = dict(workspace=str(workspace), policy=str(policy_path),
            policy_sha256=digest(policy_path), source_hashes={
                str(p.relative_to(package)): digest(p) for p in package.rglob('*')
                if p.is_file() and not {'__pycache__', '.pytest_cache'}.intersection(p.parts)},
            changes=changes)
    scenes = {}
    for split, combinations in (
        ('train', ((8, 'walls'), (16, 'obstacles'), (24, 'hidden_obstacles'))),
        ('validation', ((12, 'mixed'), (24, 'obstacles'), (32, 'hidden_obstacles'))),
    ):
        selected = []
        pool = sorted((ROOT / 'scenarios_robust/marker_sizes_v1' / split).glob('*.yaml'))
        for difficulty, profile in combinations:
            file = next(p for p in pool
                        if (yaml.safe_load(p.read_text())['difficulty'],
                            yaml.safe_load(p.read_text())['mount_profile']) == (difficulty, profile))
            cfg = yaml.safe_load(file.read_text())
            selected.append(dict(file=str(file.resolve()), seed=cfg['seed'],
                requested_obstacles=cfg['difficulty'], actual_obstacles=len(cfg['obstacles']),
                mount_profile=cfg.get('mount_profile'), sha256=digest(file)))
        scenes[split] = selected
    config = dict(run_id='nov09-slam-nav2-v1',
        created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        purpose='Joint black-box parameter search using real Gazebo SLAM/Nav2 episodes',
        environment='gazebo_harmonic', max_native_concurrency=2,
        screen_episodes=9, validation_episodes=3, validation_finalists=1,
        holdout_enabled=False, reserved_holdout_used=False,
        candidates=candidates, screen_scenes=scenes['train'], validation_scenes=scenes['validation'],
        support_hashes={str(HERE / name): digest(HERE / name) for name in (
            'native_tournament.py', 'gazebo_benchmark.py', 'audit_native_campaign.py',
            'run_native_service.py')})
    (output / 'configuration.json').write_text(json.dumps(config, indent=2) + '\n')
    print(json.dumps(dict(output=str(output), prepared_candidates=len(candidates),
                          planned_native_episodes=12, reserved_holdout_used=False)))


if __name__ == '__main__':
    main()
