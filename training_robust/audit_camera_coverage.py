"""Read-only snapshot of the exact inputs to live camera coverage."""
import argparse
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, LaserScan
from tf2_ros import Buffer, TransformListener
from osp_mission.mission import Mission
from osp_mission.surface_coverage import SurfaceCoverage


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='training_robust/native_diagnostics/coverage_inputs')
    parser.add_argument('--seconds', type=float, default=25)
    parser.add_argument('--save-every', type=int, default=10)
    parser.add_argument('--coverage', choices=['surface_endpoint', 'edge'], default='surface_endpoint')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('coverage_input_audit', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    if args.coverage == 'edge':
        from osp_mission.edge_policy import EdgeCoverage
        coverage = EdgeCoverage()
    else:
        coverage = SurfaceCoverage(directional=False)
    root = Path(args.output)
    root.mkdir(parents=True, exist_ok=True)
    rows = []

    def observe(data, resolution, origin, camera, endpoints, stamp, focal, horizontal_fov):
        before = int(coverage.viewed.sum()) if coverage.viewed is not None else 0
        geometry_changed = coverage.geometry != (data.shape, resolution, tuple(origin))
        map_changed = coverage.data is None or not np.array_equal(coverage.data, data)
        coverage.grid(data, resolution, origin)
        retained = int(coverage.viewed.sum())
        coverage.observe(data, resolution, origin, camera, endpoints, stamp, focal, horizontal_fov)
        points = np.asarray(endpoints)
        xs = np.floor((points[:, 0] - origin[0]) / resolution).astype(int)
        ys = np.floor((points[:, 1] - origin[1]) / resolution).astype(int)
        inside = (xs >= 0) & (xs < data.shape[1]) & (ys >= 0) & (ys < data.shape[0])
        xs, ys, points = xs[inside], ys[inside], points[inside]
        relative = (np.arctan2(points[:, 1]-camera[1], points[:, 0]-camera[0])-camera[2]+math.pi) % (2*math.pi)-math.pi
        in_view = np.abs(relative) < horizontal_fov/2
        occupied = data[ys, xs] > 50
        row = dict(stamp=stamp, endpoints=len(points), in_view=int(in_view.sum()), occupied=int(occupied.sum()),
                   occupied_in_view=int((in_view & occupied).sum()), viewed=int(coverage.viewed.sum()),
                   counts=int((coverage.counts > 0).sum()), camera=list(camera), fov=horizontal_fov,
                   geometry_changed=geometry_changed, map_changed=map_changed,
                   viewed_before_update=before, viewed_retained=retained,
                   lost_on_update=max(0, before-retained),
                   gained_on_frame=max(0, int(coverage.viewed.sum())-retained))
        rows.append(row)
        if (len(rows)-1) % args.save_every == 0:
            np.savez_compressed(root / f'frame_{len(rows):03}.npz', data=data, resolution=resolution,
                                origin=origin, camera=camera, endpoints=endpoints, stamp=stamp,
                                focal=focal, horizontal_fov=horizontal_fov, counts=coverage.counts)

    shadow = SimpleNamespace(map=None, latest_scan=None, pending_camera=None, last_camera_stamp=-1,
                             camera_stamp=None, surface_mode=True, tf=buffer, coverage=SimpleNamespace(observe=observe),
                             camera_diagnostics={k: 0 for k in ['received','late','invalid_calibration','duplicate','accepted','coverage_updates','tf_waits']},
                             p=lambda k: 'map', get_clock=node.get_clock, get_logger=node.get_logger)
    shadow.observe_camera = lambda: Mission.observe_camera(shadow)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    subs = [node.create_subscription(OccupancyGrid, 'map', lambda m: Mission.on_map(shadow, m), qos),
            node.create_subscription(LaserScan, 'scan', lambda m: Mission.on_scan(shadow, m), qos_profile_sensor_data),
            node.create_subscription(CameraInfo, 'mission/processed_camera', lambda m: Mission.on_camera(shadow, m), qos_profile_sensor_data)]
    start = time.monotonic()
    while time.monotonic()-start < args.seconds:
        rclpy.spin_once(node, timeout_sec=.1)
        shadow.observe_camera()
    report = dict(coverage=args.coverage, diagnostics=shadow.camera_diagnostics, frames=rows,
                  lost_on_update=sum(r['lost_on_update'] for r in rows),
                  gained_on_frames=sum(r['gained_on_frame'] for r in rows))
    (root/'report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(dict(diagnostics=shadow.camera_diagnostics, frames=len(rows), examples=rows[::max(1,len(rows)//5)]), indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
