"""Build a fresh demo track from a separately installed upstream room generator."""
import argparse
import json
import random
import secrets
from pathlib import Path
import subprocess
import sys
from xml.etree import ElementTree as ET

import cv2
import yaml


def choose_layout(template, initial_seed):
    from osp_project.demo_room import table_layout
    from osp_project.sim_world import randomize_scene

    # The upstream grid models table legs at wheel height. Keep the blue boxes
    # out from underneath the tabletops as well so their marked faces stay visible.
    tables = table_layout(template)
    retry = random.Random(initial_seed)
    seed = initial_seed
    for _ in range(64):
        scene = randomize_scene(template, seed)
        overlaps = False
        for box in scene['obstacles']:
            for table in tables:
                width, height = (table['sy'], table['sx']) if table['yaw'] else (table['sx'], table['sy'])
                if (abs(box['x']-table['x']) < (width+box['sx'])/2+.03 and
                    abs(box['y']-table['y']) < (height+box['sy'])/2+.03):
                    overlaps = True
                    break
            if overlaps:
                break
        if not overlaps:
            return scene
        seed = retry.getrandbits(32)
    raise ValueError('Could not place all boxes outside the tables')


def prepare(reference, output, seed=None):
    asset = reference / 'resources/demo_boxes_walls.yaml'
    config = yaml.safe_load(asset.read_text())
    template = config['scenes'][config['scenario']]
    template['randomization'].update(enabled=True, seed=seed, wall_marker_count=1, relocate_obstacles=True)
    # Keep the two IDs from the comparison and the upstream third target.
    for target in template['targets']:
        if target['id'] == 15:
            target['id'] = 103
    source = output / 'generator.yaml'
    source.write_text(yaml.safe_dump(config))
    chosen = choose_layout(template, secrets.randbits(32) if seed is None else seed)
    # Render exactly the verified draw; a second random draw would invalidate it.
    render = dict(chosen, randomization=dict(chosen['randomization'], enabled=False))
    render.pop('name', None)
    render_config = output / 'render.yaml'
    render_config.write_text(yaml.safe_dump({'scenario':'track', 'scenes':{'track':render}}))
    subprocess.run([sys.executable, '-m', 'osp_project.sim_world', '--config', str(render_config)],
                   cwd=output, check=True)
    generated = output / 'run/simulation'
    texture = cv2.imread(str(generated / 'demo_floor.png'))
    preview = cv2.resize(texture, None, fx=.4, fy=.4, interpolation=cv2.INTER_AREA)
    if not cv2.imwrite(str(generated / 'board-floor.jpg'), preview):
        raise OSError('Could not save the board floor preview')
    scene = json.loads((generated / 'scene.json').read_text())
    scene['randomization'] = chosen['randomization']
    targets = scene['targets']
    boxes = {body['name'] for body in scene['obstacles']}
    box_hosts = {t['attached_to'] for t in targets if t['attached_to'] in boxes}
    walls = [t for t in targets if t['attached_to'].startswith('wall_')]
    if {t['id'] for t in targets} != {7, 32, 103} or len(targets) != 3 or len(walls) != 1 or len(box_hosts) != 2:
        raise ValueError('Expected one wall marker and two different box markers')
    world = ET.parse(generated / 'room.sdf')
    node = world.find('world')
    node.set('name', 'osp_studio')
    for system in ('Contact', 'Imu'):
        ET.SubElement(node, 'plugin', filename=f'gz-sim-{system.lower()}-system', name=f'gz::sim::systems::{system}')
    destination = output / 'world'
    destination.mkdir()
    world.write(destination / 'osp_studio.sdf', encoding='utf-8', xml_declaration=True)
    (output / 'scene.json').write_text(json.dumps(scene, indent=2))
    pose = scene['robot_spawn']
    params = dict(start=[pose[k] for k in ('x', 'y', 'yaw')], duration=600, targets=targets)
    (output / 'scene.yaml').write_text(yaml.safe_dump(params))
    print('seed', scene['seed'], [(t['id'], t['attached_to']) for t in targets], flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('reference', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--seed', type=int, default=None)
    args = parser.parse_args()
    prepare(args.reference.resolve(), args.output.resolve(), args.seed)
