"""Verify a final SLAM occupancy slice against separated, timestamped LiDAR views.

Each run stays in its own map frame. No map registration or semantic table-leg
classification is implied. Pose/SLAM errors can still leave false static returns.
"""
import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
import json
import math
from pathlib import Path

import numpy as np
from scipy.ndimage import label
from scipy.spatial.transform import Rotation, Slerp

from consensus import Consensus, scan_evidence


def stamp(s):
    return s.sec + s.nanosec * 1e-9


def matrix(t):
    m = np.eye(4)
    q = t.rotation
    m[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    m[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return m


class Transforms:
    def __init__(self):
        self.static, self.dynamic = {}, defaultdict(list)

    def add(self, msg, static):
        for tr in msg.transforms:
            key = (tr.header.frame_id.lstrip('/'), tr.child_frame_id.lstrip('/'))
            if static:
                self.static[key] = matrix(tr.transform)
            else:
                self.dynamic[key].append((stamp(tr.header.stamp), matrix(tr.transform)))

    def finish(self):
        for key, rows in self.dynamic.items():
            unique = {t: m for t, m in rows}
            self.dynamic[key] = sorted(unique.items())
        self.times = {k: [t for t, _ in rows] for k, rows in self.dynamic.items()}

    def edge(self, key, when):
        if key in self.static:
            return self.static[key]
        rows, times = self.dynamic[key], self.times[key]
        i = bisect_left(times, when)
        if i == len(rows) or i == 0:
            j = min(i, len(rows)-1)
            if abs(rows[j][0]-when) > .15:
                raise ValueError('TF extrapolation exceeds 150ms')
            return rows[j][1]
        t0, m0 = rows[i-1]
        t1, m1 = rows[i]
        if min(when-t0, t1-when) > .15 or t1-t0 > .5:
            raise ValueError('TF gap exceeds allowed interpolation')
        ratio = (when-t0)/(t1-t0)
        out = np.eye(4)
        out[:3, 3] = m0[:3, 3]*(1-ratio)+m1[:3, 3]*ratio
        out[:3, :3] = Slerp([t0, t1], Rotation.from_matrix([m0[:3, :3], m1[:3, :3]]))([when]).as_matrix()[0]
        return out

    def lookup(self, target, source, when):
        keys = set(self.static) | set(self.dynamic)
        queue = [(source.lstrip('/'), np.eye(4))]
        seen = set()
        while queue:
            frame, accumulated = queue.pop(0)
            if frame == target.lstrip('/'):
                return accumulated
            if frame in seen:
                continue
            seen.add(frame)
            for parent, child in keys:
                if frame not in (parent, child):
                    continue
                try:
                    edge = self.edge((parent, child), when)
                except ValueError:
                    continue
                if child == frame:
                    queue.append((parent, edge @ accumulated))
                else:
                    queue.append((child, np.linalg.inv(edge) @ accumulated))
        raise ValueError('Missing timestamped TF chain')


def stream(folder, topics):
    import rosbag2_py as b
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = b.SequentialReader()
    reader.open(b.StorageOptions(uri=str(folder/'rosbag'), storage_id='mcap'), b.ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in topics}
    reader.set_filter(b.StorageFilter(topics=list(types)))
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        yield topic, deserialize_message(raw, types[topic])


def analyze(folder, output):
    import yaml
    tf = Transforms()
    final = None
    for topic, msg in stream(folder, ['/map', '/tf', '/tf_static']):
        if topic == '/map':
            final = msg
        else:
            tf.add(msg, topic == '/tf_static')
    if final is None:
        raise ValueError('No saved SLAM map')
    tf.finish()
    grid = np.asarray(final.data).reshape(final.info.height, final.info.width)
    occupied = grid >= 65
    rr, cc = np.nonzero(occupied)
    resolution = final.info.resolution
    origin = final.info.origin
    angle = Rotation.from_quat([origin.orientation.x, origin.orientation.y,
                               origin.orientation.z, origin.orientation.w]).as_euler('xyz')[2]
    rot = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
    xy = (np.column_stack([cc+.5, rr+.5])*resolution) @ rot.T + [origin.position.x, origin.position.y]
    cfg = yaml.safe_load((folder/'config/slam.yaml').read_text())['slam_toolbox']['ros__parameters']
    limit = cfg['max_laser_range']
    evidence = Consensus(occupied, resolution)
    rejected = Counter()
    for _, scan in stream(folder, ['/scan_self_filtered']):
        when = stamp(scan.header.stamp)
        if evidence.views and when-evidence.views[-1][0] < 2.:
            continue
        try:
            tr = tf.lookup(final.header.frame_id, scan.header.frame_id, when)
        except ValueError:
            rejected['missing_or_stale_tf'] += 1
            continue
        yaw = math.atan2(tr[1, 0], tr[0, 0])
        pose = np.r_[tr[:2, 3], yaw]
        if evidence.views:
            old = evidence.views[-1][1]
            turn = abs(math.remainder(yaw-old[2], math.tau))
            if np.linalg.norm(pose[:2]-old[:2]) < .25 and turn < math.radians(15):
                rejected['same_viewpoint'] += 1
                continue
        hit, free = scan_evidence(xy, tr[:2, 3], yaw, scan.ranges, scan.angle_min,
            scan.angle_increment, scan.range_min, scan.range_max, limit)
        h = np.zeros(grid.shape, bool)
        f = h.copy()
        h[rr, cc], f[rr, cc] = hit, free
        evidence.add(when, pose, h, f)
    persistent = evidence.mask()
    components, count = label(persistent)
    sizes = np.bincount(components.ravel())[1:]
    result = dict(run=folder.name, slam_max_range_m=limit, map_frame=final.header.frame_id,
        status='evidence_available' if evidence.views else 'unavailable_missing_timestamped_tf',
        views=len(evidence.views), occupied_cells=int(occupied.sum()),
        repeated_supported_cells=int(persistent.sum()), unverified_cells=int((occupied & ~persistent).sum()),
        components=int(count), small_components_under_0_04_m2=int(np.sum((sizes*resolution**2 <= .04))),
        rejected=dict(rejected), thresholds=dict(minimum_views=3, minimum_span_s=6.,
            minimum_view_translation_m=.25, minimum_view_turn_deg=15., range_tolerance_m=.12,
            maximum_free_to_hit_ratio=.25),
        limitations=['Repeated returns do not identify a table leg, person or box semantically',
            'Stationary people and map/TF bias can persist; this is a conservative reconstruction candidate',
            'Unverified cells are not free space; physical costmaps and collision checks are untouched',
            'Separate runs use separate map frames and are not merged',
            'Geometry is only a horizontal LiDAR slice, not complete 3D shape'])
    output.mkdir(parents=True, exist_ok=True)
    (output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    np.savez_compressed(output/'evidence.npz', grid=grid, persistent=persistent, hits=evidence.hits,
                        free=evidence.free, resolution=resolution, origin=[origin.position.x, origin.position.y, angle])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    image = np.full((*grid.shape, 3), .93)
    image[grid < 0] = [.75, .77, .8]
    image[occupied] = [.65, .65, .65]
    image[persistent] = [.1, .35, .65]
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(image, origin='lower')
    ax.set_title(f'{folder.name} | SLAM {limit:g}m | {len(evidence.views)} separated LiDAR views\nBlue: repeated support; gray: unverified (not free)')
    ax.set_axis_off()
    fig.tight_layout()
    fig.savefig(output/'persistent.png', dpi=140)
    plt.close(fig)
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    p.add_argument('output', type=Path)
    a = p.parse_args()
    print(json.dumps(analyze(a.run, a.output), indent=2))
