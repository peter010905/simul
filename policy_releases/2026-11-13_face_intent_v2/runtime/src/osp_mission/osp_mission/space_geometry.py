"""Shared physical geometry for diverse Gazebo arenas and offline validation.

Only the world generator/evaluator consumes this module. Policies see ROS
observations, never the arena polygon, target locations, or scenario seed.
"""
import math
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt, label

SHAPES = ('circle', 'triangle', 'trapezoid', 'rectangle', 'l_shape', 'u_shape', 'rooms')


def inside_polygon(x, y, vertices):
    inside = np.zeros(np.broadcast(x, y).shape, dtype=bool)
    for a, b in zip(vertices, vertices[1:] + vertices[:1]):
        if abs(b[1] - a[1]) < 1e-12:
            continue
        crossing = (a[1] > y) != (b[1] > y)
        inside ^= crossing & (x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0])
    return inside


def polygon_area(vertices):
    return sum(a[0]*b[1]-a[1]*b[0] for a, b in zip(vertices, vertices[1:]+vertices[:1])) / 2


def wall(a, b, thickness=.14, **extra):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    return dict(type='box', x=(a[0]+b[0])/2, y=(a[1]+b[1])/2,
                sx=length + .04, sy=thickness, yaw=math.atan2(dy, dx),
                height=1.4, length=length, inward=[-dy/length, dx/length], **extra)


def make_arena(rng, shape):
    if shape not in SHAPES:
        raise ValueError(shape)
    if shape == 'circle':
        width = height = float(rng.uniform(14, 20))
        radius = width / 2
        polygon = [[radius + radius*math.cos(a), radius + radius*math.sin(a)]
                   for a in np.linspace(0, 2*math.pi, 48, endpoint=False)]
    elif shape == 'triangle':
        width, height = float(rng.uniform(19, 27)), float(rng.uniform(14, 20))
        polygon = [[0., 0.], [width, 0.], [width*float(rng.uniform(.35, .65)), height]]
    else:
        width, height = float(rng.uniform(15, 25)), float(rng.uniform(10, 17))
        if shape == 'trapezoid':
            inset = width*float(rng.uniform(.15, .30))
            polygon = [[0., 0.], [width, 0.], [width-inset, height], [inset, height]]
        elif shape == 'l_shape':
            wx, hy = width*float(rng.uniform(.4, .6)), height*float(rng.uniform(.4, .6))
            polygon = [[0., 0.], [width, 0.], [width, hy], [wx, hy], [wx, height], [0., height]]
        elif shape == 'u_shape':
            wx, hy = width*float(rng.uniform(.28, .36)), height*float(rng.uniform(.30, .42))
            polygon = [[0., 0.], [width, 0.], [width, height], [width-wx, height],
                       [width-wx, hy], [wx, hy], [wx, height], [0., height]]
        else:
            polygon = [[0., 0.], [width, 0.], [width, height], [0., height]]
    walls = [wall(a, b, boundary=True) for a, b in zip(polygon, polygon[1:]+polygon[:1])]
    doors = []
    if shape == 'rooms':
        # Three rooms connected by two independently placed doorways.
        split_x = width*float(rng.uniform(.38, .58))
        split_y = height*float(rng.uniform(.42, .62))
        door_width = float(rng.uniform(1.3, 2.0))
        door_y = height*float(rng.uniform(.2, .78))
        for lo, hi in [(0, door_y-door_width/2), (door_y+door_width/2, height)]:
            if hi-lo > .1:
                walls.append(wall([split_x, lo], [split_x, hi], boundary=False))
        door_x = float(rng.uniform(split_x+1.2, width-1.2))
        for lo, hi in [(split_x, door_x-door_width/2), (door_x+door_width/2, width)]:
            if hi-lo > .1:
                walls.append(wall([lo, split_y], [hi, split_y], boundary=False))
        doors = [dict(x=split_x, y=door_y, width=door_width),
                 dict(x=door_x, y=split_y, width=door_width)]
    return dict(schema_version=1, shape=shape, width_m=width, height_m=height,
                polygon=polygon, walls=walls, doorways=doors,
                area_m2=polygon_area(polygon),
                circle_approximation_segments=48 if shape == 'circle' else None,
                pdf_fixtures=False)


def rasterize_space(cfg, resolution=.15, include_supports=True):
    arena = cfg['arena']
    rows = math.ceil(arena['height_m']/resolution)
    cols = math.ceil(arena['width_m']/resolution)
    yy, xx = np.indices((rows, cols))
    x, y = (xx+.5)*resolution, (yy+.5)*resolution
    occupied = ~inside_polygon(x, y, arena['polygon'])
    objects = list(arena['walls']) + list(cfg.get('obstacles', []))
    if include_supports:
        for t in cfg.get('targets', []):
            objects.append(dict(type='box', x=t['x']-.05*math.cos(t['yaw']),
                y=t['y']-.05*math.sin(t['yaw']), sx=.08, sy=1.25*t['size'], yaw=t['yaw']))
    for obj in objects:
        dx, dy = x-obj['x'], y-obj['y']
        if obj['type'] == 'cylinder':
            occupied |= dx*dx+dy*dy <= (obj['radius']+resolution*.45)**2
        else:
            angle = obj.get('yaw', 0)
            u, v = dx*math.cos(angle)+dy*math.sin(angle), -dx*math.sin(angle)+dy*math.cos(angle)
            occupied |= (np.abs(u) <= obj['sx']/2+resolution*.45) & (np.abs(v) <= obj['sy']/2+resolution*.45)
    return occupied


def accessible_space(cfg, resolution=.15):
    occupied = rasterize_space(cfg, resolution)
    safe = distance_transform_edt(~occupied)*resolution >= .34
    groups, _ = label(safe)
    x, y = cfg['start'][:2]
    row, col = int(y/resolution), int(x/resolution)
    if not (0 <= row < safe.shape[0] and 0 <= col < safe.shape[1]) or not groups[row, col]:
        return np.zeros_like(safe)
    return safe & (groups == groups[row, col])


def valid_space(cfg, resolution=.15):
    occupied = rasterize_space(cfg, resolution)
    safe = distance_transform_edt(~occupied)*resolution >= .34
    accessible = accessible_space(cfg, resolution)
    # Reject disconnected rooms or obstacle-created islands, not just bad starts.
    if accessible.sum() < max(45/resolution**2, .97*safe.sum()):
        return False
    for target in cfg['targets']:
        found = False
        for distance in (.85, 1.1, 1.35):
            ds = np.linspace(.20, distance, 20)
            xs = np.floor((target['x']+ds*math.cos(target['yaw']))/resolution).astype(int)
            ys = np.floor((target['y']+ds*math.sin(target['yaw']))/resolution).astype(int)
            if np.any(xs < 0) or np.any(ys < 0) or np.any(xs >= safe.shape[1]) or np.any(ys >= safe.shape[0]):
                continue
            if accessible[ys[-1], xs[-1]] and not occupied[ys, xs].any():
                found = True
                break
        if not found:
            return False
    return True


def triangulate(polygon):
    """Ear clipping for simple, counter-clockwise arena polygons."""
    vertices = np.asarray(polygon, dtype=float)
    remaining, faces = list(range(len(vertices))), []
    def cross(a, b, c):
        return np.cross(b-a, c-a).item()
    while len(remaining) > 3:
        for i, current in enumerate(remaining):
            before, after = remaining[i-1], remaining[(i+1) % len(remaining)]
            a, b, c = vertices[[before, current, after]]
            if cross(a, b, c) <= 1e-9:
                continue
            if any(cross(a, b, vertices[j]) >= -1e-9 and cross(b, c, vertices[j]) >= -1e-9
                   and cross(c, a, vertices[j]) >= -1e-9 for j in remaining if j not in (before, current, after)):
                continue
            faces.append((before, current, after))
            remaining.pop(i)
            break
        else:
            raise ValueError('Invalid or self-intersecting arena polygon')
    faces.append(tuple(remaining))
    return faces


def write_floor_mesh(arena, output):
    path = Path(output) / 'arena_floor.obj'
    faces = triangulate(arena['polygon'])
    lines = [f'v {x} {y} 0' for x, y in arena['polygon']]
    lines += ['f ' + ' '.join(str(index+1) for index in face) for face in faces]
    path.write_text('\n'.join(lines)+'\n')
    return path
