"""Accessible native worlds with different exterior shape and room topology."""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

from osp_mission.marker_sizes import randomize_sizes
from osp_mission.space_geometry import make_arena, rasterize_space, valid_space


def make_space_scene(seed, count=12, profile='mixed', shape='circle'):
    if profile not in ('walls', 'mixed', 'obstacles', 'hidden_obstacles'):
        raise ValueError(profile)
    rng = np.random.default_rng(seed)
    for attempt in range(120):
        arena = make_arena(rng, shape)
        cfg = dict(seed=int(seed), randomize=False, task='spatial_native_search_v1',
                   arena=arena, difficulty=int(count), mount_profile=profile,
                   obstacles=[], targets=[], start=[1., 1., 0.])
        base = rasterize_space(cfg, .15)
        clear = distance_transform_edt(~base)*.15
        ys, xs = np.where(clear >= .85)
        for j in range(count):
            kind = ('box', 'cylinder', 'rack', 'partition')[j % 4]
            for _ in range(160):
                index = int(rng.integers(len(xs)))
                x, y = float((xs[index]+.5)*.15), float((ys[index]+.5)*.15)
                radius = float(rng.uniform(.18, .38))
                if kind == 'box':
                    sx, sy = float(rng.uniform(.4, .95)), float(rng.uniform(.4, .95))
                elif kind == 'rack':
                    sx, sy = float(rng.uniform(1, 2.1)), .5
                else:
                    sx, sy = float(rng.uniform(1.2, 2.5)), .16
                extent = radius if kind == 'cylinder' else math.hypot(sx, sy)/2
                if clear[ys[index], xs[index]] < extent+.35:
                    continue
                if any(math.hypot(x-o['x'], y-o['y']) < extent+o['extent']+.4 for o in cfg['obstacles']):
                    continue
                cfg['obstacles'].append(dict(type='cylinder' if kind == 'cylinder' else 'box',
                    kind=kind, x=x, y=y, sx=sx, sy=sy, radius=radius,
                    yaw=float(rng.uniform(-math.pi, math.pi)), height=float(rng.uniform(.7, 1.3)), extent=extent))
                break
        if len(cfg['obstacles']) < max(3, int(.8*count)):
            continue
        free = ~rasterize_space(cfg, .15)
        ys, xs = np.where(distance_transform_edt(free)*.15 >= .5)
        if not len(xs):
            continue
        start = int(rng.integers(len(xs)))
        cfg['start'] = [float((xs[start]+.5)*.15), float((ys[start]+.5)*.15), float(rng.uniform(-math.pi, math.pi))]
        wall_count = {'walls': 3, 'mixed': 1, 'obstacles': 0, 'hidden_obstacles': 0}[profile]
        wall_indices = rng.choice(len(arena['walls']), wall_count, replace=False)
        for index in wall_indices:
            obj = arena['walls'][int(index)]
            # For the circle, use the face centre so even the largest backing fits.
            fraction = 0 if obj['length'] < 1.5 else float(rng.uniform(-.3, .3))
            nx, ny = obj['inward']
            a = math.atan2(ny, nx)
            offset = obj['sy']/2+.055
            cfg['targets'].append(dict(x=obj['x']+fraction*obj['length']*math.cos(obj['yaw'])+offset*nx,
                y=obj['y']+fraction*obj['length']*math.sin(obj['yaw'])+offset*ny,
                yaw=a, size=.12, mount='wall', carrier_wall=int(index)))
        for index in rng.choice(len(cfg['obstacles']), 3-wall_count, replace=False):
            obj = cfg['obstacles'][int(index)]
            away = math.atan2(obj['y']-cfg['start'][1], obj['x']-cfg['start'][0])
            if obj['type'] == 'cylinder':
                a, distance = away if profile == 'hidden_obstacles' else float(rng.uniform(-math.pi, math.pi)), obj['radius']
            else:
                faces = [(obj['yaw'], obj['sx']/2, obj['sy']), (obj['yaw']+math.pi, obj['sx']/2, obj['sy']),
                         (obj['yaw']+math.pi/2, obj['sy']/2, obj['sx']), (obj['yaw']-math.pi/2, obj['sy']/2, obj['sx'])]
                faces = [face for face in faces if face[2] >= .25]
                face = max(faces, key=lambda f: math.cos(f[0]-away)) if profile == 'hidden_obstacles' else faces[int(rng.integers(len(faces)))]
                a, distance = face[:2]
            cfg['targets'].append(dict(x=obj['x']+(distance+.055)*math.cos(a),
                y=obj['y']+(distance+.055)*math.sin(a), yaw=a, size=.12,
                mount='obstacle', carrier_obstacle=int(index), hidden_from_start=profile == 'hidden_obstacles'))
        for target, identifier in zip(cfg['targets'], rng.choice(1024, 3, replace=False)):
            target['id'] = int(identifier)
        cfg = randomize_sizes(cfg, seed+20000000+attempt, .08, .30)
        if profile == 'hidden_obstacles' and any(
            (cfg['start'][0]-t['x'])*math.cos(t['yaw'])+(cfg['start'][1]-t['y'])*math.sin(t['yaw']) > -.2 for t in cfg['targets']):
            continue
        if valid_space(cfg):
            for obj in cfg['obstacles']:
                obj.pop('extent')
            cfg['actual_obstacles'] = len(cfg['obstacles'])
            cfg['environment_randomization'] = dict(physical_layout_seed=int(seed), generation_attempt=attempt,
                outer_boundary=shape, fresh_training_map=True,
                randomized=['outer_shape', 'dimensions', 'room_connectivity', 'obstacles', 'mounts',
                            'ids', 'marker_sizes', 'start_position', 'start_yaw'])
            return cfg
    raise RuntimeError(f'No accessible space for {seed}/{shape}/{count}/{profile}')
