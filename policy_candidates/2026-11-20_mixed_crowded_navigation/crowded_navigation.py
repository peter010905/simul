"""Choose bounded local repositioning on a common SLAM/Nav2 planning graph."""
from collections import deque
import math

import numpy as np
from scipy.ndimage import distance_transform_edt, label, uniform_filter
from scipy.sparse.csgraph import dijkstra

from osp_mission.escape_connectivity import connect_start
from osp_mission.near_start import open_start_region, validate_radius
from osp_mission.policy import graph
from osp_mission._r32_navigation_core import nav_traversable
from osp_mission.transit_control import TransitFlow


class CommonRoutes:
    def __init__(self, grid, costmap, pose, start_ignore=0.):
        if (grid.resolution <= 0 or costmap.resolution <= 0
                or not np.isfinite([grid.resolution,costmap.resolution,*grid.origin,*costmap.origin,*pose]).all()):
            raise ValueError('Finite geometry and positive map resolutions are required')
        self.grid = grid
        self.pose = pose
        self.clearance = distance_transform_edt(grid.cells <= 50) * grid.resolution
        normal = (grid.cells == 0) & (self.clearance >= .32)
        self.normal = normal
        start = grid.index(*pose[:2])
        self.start = start
        self.distance = np.full(grid.cells.shape, np.inf)
        self.connected = np.zeros(grid.cells.shape, bool)
        if not grid.inside(*start):
            return
        radius = validate_radius(start_ignore)
        if radius:
            ox, oy, yaw = grid.origin
            dx, dy = pose[0]-ox, pose[1]-oy
            local = (math.cos(yaw)*dx+math.sin(yaw)*dy,
                     -math.sin(yaw)*dx+math.cos(yaw)*dy)
            permitted = open_start_region(normal, grid.resolution, (0.,0.), local, radius)
        else:
            permitted = connect_start(grid.cells, grid.resolution, start, normal)
        # Resample the Nav2 mask using the full origin rotation and resolution.
        rows, cols = np.indices(grid.cells.shape)
        gx, gy = (cols+.5)*grid.resolution, (rows+.5)*grid.resolution
        ox, oy, yaw = grid.origin
        x = ox+math.cos(yaw)*gx-math.sin(yaw)*gy
        y = oy+math.sin(yaw)*gx+math.cos(yaw)*gy
        ox, oy, yaw = costmap.origin
        dx, dy = x-ox, y-oy
        nc = np.floor((math.cos(yaw)*dx+math.sin(yaw)*dy)/costmap.resolution).astype(int)
        nr = np.floor((-math.sin(yaw)*dx+math.cos(yaw)*dy)/costmap.resolution).astype(int)
        inside = (nr>=0)&(nc>=0)&(nr<costmap.cells.shape[0])&(nc<costmap.cells.shape[1])
        nav = np.zeros(grid.cells.shape, bool)
        nav[inside] = nav_traversable(costmap)[nr[inside],nc[inside]]
        common = permitted & nav
        if not common[start]:
            return
        groups, _ = label(common)  # Four-neighbour connectivity.
        self.connected = common & (groups == groups[start])
        self.distance = dijkstra(graph(self.connected, grid.resolution), directed=False,
                                 indices=start[0]*grid.cells.shape[1]+start[1]).reshape(grid.cells.shape)

    def allows(self, goal):
        if goal.get('stationary'):
            return math.dist((goal['x'],goal['y']), self.pose[:2]) <= .10
        cell = self.grid.index(goal['x'], goal['y'])
        return (self.grid.inside(*cell) and bool(self.normal[cell])
                and bool(self.connected[cell]) and np.isfinite(self.distance[cell]))

    def reposition(self, veto, visits, preferred_heading=None):
        mask = self.normal & self.connected & (self.distance >= .45) & (self.distance <= 1.6)
        openness = uniform_filter(self.connected.astype(float),
                                  size=max(3,int(1.5/self.grid.resolution)|1),mode='constant')
        buckets = {}
        for row,col in zip(*np.where(mask)):
            point = self.grid.world(row,col)
            dx,dy = point-np.asarray(self.pose[:2])
            direct = math.hypot(dx,dy)
            if not .45 <= direct <= 1.0:
                continue
            yaw = math.atan2(dy,dx)
            goal = dict(x=float(point[0]),y=float(point[1]),yaw=yaw,scan_angle=0.,
                        stationary=False,cell=(int(row),int(col)),distance=float(self.distance[row,col]),
                        features=np.zeros(5),r32_kind='crowded_reposition',crowded_reposition=True)
            if veto(goal):
                continue
            revisit = sum(math.exp(-math.dist(point,p)**2/.25) for p in visits)
            turn = abs(math.remainder(yaw-self.pose[2],2*math.pi))
            alignment = math.cos(yaw-preferred_heading) if preferred_heading is not None else 0.
            score = (1.5*min(float(self.clearance[row,col]),1.) + 1.5*openness[row,col]
                     - .25*self.distance[row,col] + .30*direct - .08*turn + .35*alignment - revisit)
            key = (math.floor(point[0]/.25),math.floor(point[1]/.25))
            if key not in buckets or score>buckets[key][0]:
                buckets[key] = score,goal
        return max(buckets.values(),key=lambda item:item[0])[1] if buckets else None


class CrowdedFlow(TransitFlow):
    def __init__(self, *args):
        super().__init__(*args)
        self.position = None
        self.position_time = 0.
        self.path_sites = deque(maxlen=128)
        self.failed_sites = deque(maxlen=32)
        self.escape_anchor = None
        self.last_survey = None
        self.repositions = deque(maxlen=24)
        self.planning_reason = ''
        self.common_reachable_cells = 0
        self.route_rejections = 0

    def observe_position(self, x, y, now):
        if not np.isfinite([x,y,now]).all():
            return
        super().observe_position(x,y,now)
        self.position = (x,y)
        self.position_time = now
        if not self.path_sites or math.dist(self.position,self.path_sites[-1][1])>=.2:
            self.path_sites.append((now,self.position))
        if self.escape_anchor is not None and math.dist(self.position,self.escape_anchor)>=.4:
            self.escape_anchor = None
            self.failed_sites.clear()

    def record_navigation_result(self, status, reason, now):
        super().record_navigation_result(status,reason,now)
        failed = status==6 or (status==5 and reason in ('navigation_stalled','action_timeout','motor_stall','bump_contact'))
        if failed and self.position is not None:
            self.failed_sites.append((now,self.position))
            nearby = [p for t,p in self.failed_sites if 0<=now-t<=45 and math.dist(p,self.position)<.5]
            if len(nearby)>=2 and self.escape_anchor is None:
                self.escape_anchor = self.position

    def survey_allowed(self):
        if self.position is None:
            return False
        if self.last_survey is not None and math.dist(self.last_survey,self.position)<.4:
            return False
        self.last_survey = self.position
        return True

    def reposition_visits(self, transform):
        if transform is None:
            return []
        from osp_mission.r32_drive import in_map
        points = list(self.repositions)+[p for t,p in self.path_sites if 0<=self.position_time-t<=90.]
        return [in_map(('odom',*point),transform) for point in points]

    def dispatched_reposition(self, goal, transform):
        point = self.goal_point(goal,transform)
        if point[0]=='odom':
            self.repositions.append(point[1:])

    def report(self):
        return dict(super().report(),planning_reason=self.planning_reason,
                    common_reachable_cells=self.common_reachable_cells,
                    common_route_rejections=self.route_rejections,
                    crowded_escape_pending=self.escape_anchor is not None)

    def resolve(self, selected, choices, routes, transform, now, weights, choose):
        if routes is None:
            self.planning_reason = 'waiting_for_shared_costmap'
            self.common_reachable_cells = 0
            return None
        self.common_reachable_cells = int(np.count_nonzero(routes.connected))
        def permitted(goal):
            return routes.allows(goal) and not self.blocked(goal,transform,now)
        available = [goal for goal in choices if permitted(goal)]
        if selected is not None and not permitted(selected):
            self.route_rejections += 1
            selected = choose(available,weights)
        observing_marker = bool(selected and selected.get('stationary')
                                and any(k in selected for k in ('approach_id','focus_id','print_point')))
        if selected is None or (self.escape_anchor is not None and not observing_marker):
            preferred_heading = None
            if selected is not None and not selected.get('stationary'):
                preferred_heading = math.atan2(selected['y']-routes.pose[1],selected['x']-routes.pose[0])
            local = routes.reposition(lambda goal:not permitted(goal),
                                      self.reposition_visits(transform),preferred_heading)
            if local is not None:
                self.planning_reason = 'local_reposition'
                return local
        self.planning_reason = 'shared_route_selected' if selected is not None else 'no_shared_safe_goal'
        return selected
