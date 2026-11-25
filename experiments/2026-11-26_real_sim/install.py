"""Create an opt-in simulation-only copy; never change a physical runtime."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import yaml


HERE = Path(__file__).resolve().parent


def install(parent, destination, profile):
    parent, destination = parent.resolve(), destination.resolve()
    if destination.exists():
        raise FileExistsError(destination)
    data = json.loads(profile.read_text())
    if data['physical_drive'] is not False or data['slam_max_range_m'] not in (5., 12.):
        raise ValueError('Expected measured simulation-only profile')
    source = parent/'src/osp_mission/osp_mission/sensor_model.py'
    text = source.read_text()
    if text.count('def configure_sensors(') != 1:
        raise ValueError('Unexpected sensor configurator')
    destination.mkdir(parents=True)
    for name in ('src', 'frozen-config'):
        shutil.copytree(parent/name, destination/name, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    target = destination/'src/osp_mission/osp_mission'
    (target/'sensor_model.py').write_text(text.replace('def configure_sensors(', 'def _configure_base_sensors(')+'''

def configure_sensors(doc, camera_width=640, control_params=None, fuse_imu=False):
    import json
    import os
    from pathlib import Path
    result = _configure_base_sensors(doc, camera_width, control_params, fuse_imu)
    path = os.environ.get('OSP_REAL_SIM_PROFILE')
    if path:
        if os.environ.get('ROS_DOMAIN_ID') not in ('173', '174', '175', '185'):
            raise RuntimeError('Measured profile is restricted to isolated simulation domains')
        from osp_mission.real_sim_profile import apply
        result = apply(result, json.loads(Path(path).read_text()))
    return result
''')
    shutil.copyfile(HERE/'profile.py', target/'real_sim_profile.py')
    shutil.copyfile(profile, destination/'real_sim_profile.json')
    config = destination/'src/osp_mission/config/slam.yaml'
    slam = yaml.safe_load(config.read_text())
    slam['slam_toolbox']['ros__parameters']['max_laser_range'] = data['slam_max_range_m']
    config.write_text(yaml.safe_dump(slam))
    files = [p for p in (parent/'src/osp_mission').rglob('*')
             if p.is_file() and p.suffix in ('.py', '.json', '.yaml') and '__pycache__' not in p.parts]
    manifest = dict(physical_drive=False, parent=str(parent),
        parent_sources={str(p.relative_to(parent)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        profile_sha256=hashlib.sha256(profile.read_bytes()).hexdigest(), profile=data,
        policy_code_changed=False, sensor_generation_changed=True,
        explicit_activation='OSP_REAL_SIM_PROFILE=<workspace>/real_sim_profile.json')
    (destination/'real_sim_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return destination


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('parent', type=Path)
    p.add_argument('destination', type=Path)
    p.add_argument('profile', type=Path)
    a = p.parse_args()
    print(install(a.parent, a.destination, a.profile))
