"""Bridge R32 exploration to the existing mission's navigation action."""
import math
import numpy as np
from osp_mission import _r32_navigation_core as r32
from osp_mission.edge_policy import boundary_faces


def grid_from_message(msg):
    info = msg.info
    q = info.origin.orientation
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    cells = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
    return r32.Grid(cells, info.resolution, (info.origin.position.x, info.origin.position.y, yaw))


def in_map(point, transform):
    frame, x, y = point
    if frame == 'map':
        return (x, y)
    if transform is None:
        return None
    tx, ty, theta = transform
    c, s = math.cos(theta), math.sin(theta)
    return (c*(x-tx)+s*(y-ty), -s*(x-tx)+c*(y-ty))


class R32Drive:
    def __init__(self):
        self.costmap = None
        self.detours = 0
        self.diagnostic = {}

    def update_costmap(self, message):
        self.costmap = message

    def choose(self, mission, data, viewed, pose, transform):
        msg = self.costmap
        stamp = msg.header.stamp if msg is not None else None
        age = (mission.get_clock().now().nanoseconds/1e9 - stamp.sec - stamp.nanosec/1e9
               if stamp is not None else math.inf)
        if msg is None or msg.header.frame_id != mission.p('map_frame') or not -.1 <= age <= 3.:
            self.diagnostic = dict(reason='waiting_for_global_costmap')
            return None
        grid = grid_from_message(mission.map)
        # Include the same known dock geometry as the mission planner.
        grid = r32.Grid(data, grid.resolution, grid.origin)
        faces = boundary_faces(data, grid.resolution)
        seen = viewed[faces['side'], faces['y'], faces['x']]
        local_xy = np.column_stack((faces['px'][seen], faces['py'][seen])) * grid.resolution
        normals = np.column_stack((faces['nx'][seen], faces['ny'][seen]))
        c, s = math.cos(grid.origin[2]), math.sin(grid.origin[2])
        rotate = np.array([[c, s], [-s, c]])
        observations = np.column_stack((local_xy @ rotate + grid.origin[:2], normals @ rotate))
        now = mission.now()
        memory = mission.exploration_memory
        visited = [in_map(('odom', v['x'], v['y']), transform) for v in memory.views if now-v['stamp'] < 120.]
        blocked = [in_map(point, transform) for point, expiry in mission.navigation_flow.failures if expiry > now]
        goal, info = r32.plan_mission_goal(
            grid, pose, .30, [], [p for p in visited if p is not None],
            [p for p in blocked if p is not None], standoff=1.8, max_range=2.4,
            gain_cap=3., observed_surfaces=observations, simple_remaining=mission.p('duration')-(now-mission.started),
            frontier_block_radius=.18, detour_costmap=grid_from_message(msg), detour_side='right',
            allow_detour=self.detours < 3, allow_look=False)
        self.diagnostic = dict(policy='r32', kind=goal[3] if goal else 'no_route',
                               reachable=int(info.get('nav2_reachable_goal_cells', 0)),
                               feasible=int(info.get('feasible_candidates', 0)))
        if goal is None:
            return None
        x, y, yaw, kind = goal
        distance = float(info.get('goal_distance_m', math.hypot(x-pose[0], y-pose[1])))
        return dict(x=float(x), y=float(y), yaw=float(yaw), cell=grid.index(x,y), scan_angle=0.,
                    features=np.array([0., 0., distance, 0., 0.]), r32_kind=kind, distance=distance)

    def dispatched(self, selected):
        if selected.get('r32_kind') == 'detour':
            self.detours += 1
        else:
            self.detours = 0
