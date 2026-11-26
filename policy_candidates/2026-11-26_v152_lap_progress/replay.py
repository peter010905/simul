"""Replay recorded wall-lap decisions of a real run with parent and candidate WallLap.

For each saved decision snapshot the recorded SLAM map, map->odom transform
(TF, or a same-time odom/pose pair), global/local costmaps and robot pose are
restored. Each policy is chained on its own previous replayed goal heading and
the run's earlier dispatched lap goals are treated as credited stops. Vetoes,
marker tracking and Nav2 outcomes are not restored. No ROS node is created.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import rosbag2_py as bag
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from osp_mission.r32_drive import grid_from_message
from osp_mission._r32_navigation_core import Grid, connected_nav_cells
from osp_mission.box_access import box_routes


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.WallLap


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def snapshots(run):
    start = next(r['wall_epoch_s'] for r in map(json.loads, (run/'artifacts/startup.jsonl').read_text().splitlines())
                 if r['event'] == 'mission_started')
    reader = bag.SequentialReader()
    reader.open(bag.StorageOptions(uri=str(run/'rosbag'), storage_id='mcap'), bag.ConverterOptions('', ''))
    wanted = {'/tf', '/odom', '/global_costmap/costmap', '/local_costmap/costmap'}
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in wanted}
    reader.set_filter(bag.StorageFilter(topics=list(types)))

    def stream():
        while reader.has_next():
            n, r, t = reader.read_next()
            yield n, deserialize_message(r, types[n]), t/1e9-start
    it = iter(stream())
    msg = next(it, None)
    costs, odom, transform = {}, None, None
    for f in sorted((run/'artifacts/policy_decisions').glob('*.npz')):
        a = np.load(f)
        elapsed = float(a['elapsed'])
        while msg and msg[2] <= elapsed:
            name, m, t = msg
            if name == '/tf':
                for tr in m.transforms:
                    if (tr.header.frame_id, tr.child_frame_id) == ('map', 'odom'):
                        h = yaw_of(tr.transform.rotation)
                        c, s = math.cos(h), math.sin(h)
                        x, y = tr.transform.translation.x, tr.transform.translation.y
                        transform = (-c*x-s*y, s*x-c*y, -h)
            elif name == '/odom':
                odom = (t, m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
            else:
                costs[name] = (m.header.frame_id, grid_from_message(m))
            msg = next(it, None)
        used = transform
        if used is None and odom and abs(odom[0]-elapsed) <= .15:
            h = odom[3]-float(a['heading'])
            c, s = math.cos(h), math.sin(h)
            x, y = a['robot']
            used = (odom[1]-c*x+s*y, odom[2]-s*x-c*y, h)
        if used is None or '/global_costmap/costmap' not in costs:
            continue
        yield f.name, a, elapsed, used, costs['/global_costmap/costmap'][1], costs.get('/local_costmap/costmap')


def to_odom(point, transform):
    # transform maps map->odom as used by osp_mission.box_views.odom_point
    x, y, h = transform
    c, s = math.cos(h), math.sin(h)
    return (c*point[0]-s*point[1]+x, s*point[0]+c*point[1]+y)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    p.add_argument('parent', type=Path)
    p.add_argument('candidate', type=Path)
    a = p.parse_args()
    actual = [json.loads(l) for l in (a.run/'artifacts/hybrid_goals.jsonl').read_text().splitlines()]
    actual = [g for g in actual if g['kind'] == 'wall_lap']
    policies = {'parent': load(a.parent, 'parent_lap'), 'candidate': load(a.candidate, 'candidate_lap')}
    heading = {k: None for k in policies}
    rows = []
    previous_robot = None
    for index, (name, snap, elapsed, transform, cost, local) in enumerate(snapshots(a.run)):
        g = Grid(snap['data'], float(snap['resolution']), (*snap['origin'], 0.))
        pose = (*snap['robot'], float(snap['heading']))
        safe, base = g.reachable(pose[:2], .32)
        nav = connected_nav_cells(cost, pose)
        _, dist = box_routes(g, cost, nav, safe, pose, transform, local, departure=np.isfinite(base))
        robot_odom = to_odom(pose[:2], transform)
        travel = None
        if previous_robot is not None and math.dist(robot_odom, previous_robot) > .3:
            travel = math.atan2(robot_odom[1]-previous_robot[1], robot_odom[0]-previous_robot[0])
        row = dict(snapshot=name, elapsed=round(elapsed, 1), robot_odom=[round(v, 2) for v in robot_odom],
                   actual_goal=[round(actual[index]['x'], 2), round(actual[index]['y'], 2)] if index < len(actual) else None,
                   travel_deg=None if travel is None else round(math.degrees(travel), 1))
        for key, WallLap in policies.items():
            lap = WallLap()
            if index > 0:
                lap.entry = tuple(actual[0][k] for k in ('x', 'y'))
                lap.visited = [(r['x'], r['y']) for r in actual[:index]]
                lap.tangent_odom = heading[key] if heading[key] is not None else travel
            goal = lap.choose(g, dist, pose, transform, elapsed, 600., lambda _: False)
            out = dict(reason=lap.reason, mode=lap.selection_mode)
            if goal is not None:
                gp = goal['lap_pose_odom']
                yaw = goal['lap_tangent_odom']
                heading[key] = yaw
                out.update(goal_odom=[round(v, 2) for v in gp], yaw_deg=round(math.degrees(yaw), 1))
                if travel is not None:
                    d = np.subtract(gp, robot_odom)
                    out['ahead_m'] = round(float(d @ [math.cos(travel), math.sin(travel)]), 2)
                    out['yaw_vs_travel_deg'] = round(abs(math.degrees(math.remainder(yaw-travel, 2*math.pi))), 1)
                move = math.atan2(gp[1]-robot_odom[1], gp[0]-robot_odom[0])
                out['yaw_vs_move_deg'] = round(abs(math.degrees(math.remainder(yaw-move, 2*math.pi))), 1)
            row[key] = out
        rows.append(row)
        previous_robot = robot_odom
    summary = {}
    for key in policies:
        rs = [r[key] for r in rows if 'goal_odom' in r[key]]
        summary[key] = dict(goals=len(rs),
                            behind_travel=sum(1 for r in rs if r.get('ahead_m') is not None and r['ahead_m'] < -.30),
                            reversed_heading=sum(1 for r in rs if r['yaw_vs_move_deg'] > 135.),
                            mean_yaw_vs_move_deg=round(float(np.mean([r['yaw_vs_move_deg'] for r in rs])), 1) if rs else None)
    print(json.dumps(dict(run=a.run.name, rows=rows, summary=summary), indent=1))


if __name__ == '__main__':
    main()
