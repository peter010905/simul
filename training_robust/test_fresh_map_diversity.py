import copy
import hashlib
import json

import yaml

from audit_fresh_map_diversity import audit, occupancy_hash


def scene(seed, obstacle_x):
    return dict(seed=seed, difficulty=8, mount_profile='obstacles', start=[1.2, 1.3, 0.],
        obstacles=[dict(type='box', x=obstacle_x, y=4., sx=1., sy=1., height=1., yaw=0.)],
        targets=[dict(id=seed, x=5., y=5., size=.2, yaw=0.)])


def test_identity_and_seed_changes_do_not_count_as_new_occupancy():
    first = scene(7, 4.)
    second = copy.deepcopy(first)
    second['seed'] = 8
    second['targets'][0]['id'] = 8
    assert occupancy_hash(first) == occupancy_hash(second)
    assert occupancy_hash(first) != occupancy_hash(scene(8, 8.))


def test_audit_detects_repeated_geometry_despite_unique_seeds(tmp_path):
    jobs = []
    for number in range(2):
        path = tmp_path / f'{number}.yaml'
        path.write_text(yaml.safe_dump(scene(number, 4.)))
        jobs.append(dict(number=number, phase='train', generation=0,
            scenario=dict(file=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())))
    (tmp_path/'state.json').write_text(json.dumps(dict(jobs=jobs)))
    result = audit(tmp_path)
    assert result['training_episodes'] == 2
    assert result['unique_occupancy_maps'] == 1
    assert result['no_reused_occupancy'] is False
