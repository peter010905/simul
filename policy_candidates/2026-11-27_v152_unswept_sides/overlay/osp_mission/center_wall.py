"""Choose inspection poses from live SLAM, camera coverage and the Nav2 costmap."""
import math
from collections import deque

import numpy as np
from scipy import ndimage

from osp_mission import _r32_navigation_core as r32
from osp_mission.adaptive_faces import inspection_pose, partial_face_pose
from osp_mission.box_views import odom_point
from osp_mission.edge_policy import DIRECTIONS, inside_coverage_range
from osp_mission.tracked_faces import TrackedFaces
from osp_mission.cluster_exit import ClusterExit
from osp_mission.glass_wall_mask import GlassWallMask
from osp_mission.marker_cooldown import exploration_gain
from osp_mission.white_legs import ThinObstacles
from osp_mission.small_return_filter import SmallReturns
from osp_mission.box_batch import BoxBatch
from osp_mission.box_continuity import BoxContinuity
from osp_mission.interior_priority import InteriorPriority, hinted
from osp_mission.work_rules import RULES, FaceWork
from osp_mission.wall_lap import WallLap
from osp_mission.region_transit import RegionTransit, route_connected
from osp_mission.box_access import box_routes
from osp_mission.box_face_recovery import BoxFaceRecovery
from osp_mission.marked_boxes import MarkedBoxes
from osp_mission.wall_bearings import WallBearings
from osp_mission.visual_survey import VisualSurvey
from osp_mission.regional_surface import RegionalSurface
from osp_mission.bearing_advance import BearingAdvance
from osp_mission.open_regions import choose_unvisited


def partly_mapped_objects(labels, boxes, floor, resolution):
    """Recognize finite occupied returns with observed floor around their flanks."""
    objects = set()
    angles = np.arange(16) * (2 * math.pi / 16)
    directions = np.stack((np.sin(angles), np.cos(angles)), axis=1)
    for ident, box in enumerate(boxes, 1):
        if box is None:
            continue
        extent = max(s.stop-s.start for s in box) * resolution
        if not .35 <= extent <= 2.4:
            continue
        points = np.argwhere(labels[box] == ident)
        if len(points) < 4:
            continue
        points += [s.start for s in box]
        center = points.mean(axis=0)
        radii = extent/2 + np.array([.20, .40, .60])
        ring = np.rint(center + directions[:, None, :] * radii[None, :, None]/resolution).astype(int)
        rows, cols = ring.transpose(2, 0, 1)
        inside = (rows >= 0) & (rows < floor.shape[0]) & (cols >= 0) & (cols < floor.shape[1])
        known = np.zeros(inside.shape, bool)
        known[inside] = floor[rows[inside], cols[inside]]
        open_sides = known.sum(axis=1) >= 2
        if open_sides.sum() < 7:
            continue
        values, axes = np.linalg.eigh(np.cov(points.T) * resolution**2)
        # A short straight fragment of a room wall can have free space past
        # both ends. Require free space across its thickness, or a measured
        # corner with floor on opposed sides of the component.
        opposite = open_sides[:8] & open_sides[8:]
        across = np.abs(directions[:8] @ axes[:, 0]) > .7
        if np.any(opposite & across) or (values[0] >= .065**2 and opposite.any()):
            objects.add(ident)
    return objects


def discovery_frontier(choices, costmap, nav, veto):
    frontier = None
    best_score = -np.inf
    for candidate in choices:
        gain = candidate['features'][1]
        if gain < .15 or candidate.get('stationary') or veto(candidate):
            continue
        cell = costmap.index(candidate['x'], candidate['y'])
        if not costmap.inside(*cell) or not nav[cell]:
            continue
        travel = candidate['distance']
        # Prefer an informative new position over successive tiny steps
        # along the same perimeter. Flow already checked SLAM path safety.
        novelty = candidate['features'][4]
        score = gain/(1. + .18*travel + max(0., 2.-travel) + novelty)
        if score > best_score:
            best_score = score
            frontier = dict(candidate, r32_kind='interior_frontier')
    return frontier


def wall_sized_distance(grid, length=2.0):
    """.63: metres from each cell to occupied structure at least `length` long (no box is)."""
    occupied, _ = ndimage.label(grid.cells >= 65, structure=np.ones((3, 3)))
    spans = ndimage.find_objects(occupied)
    long_ids = [i+1 for i, s in enumerate(spans) if s is not None
                and max(s[0].stop-s[0].start, s[1].stop-s[1].start)*grid.resolution >= length]
    if not long_ids:
        return None
    return ndimage.distance_transform_edt(~np.isin(occupied, long_ids))*grid.resolution


def on_wall(grid, wall_distance, point, margin=.20):
    if wall_distance is None:
        return False
    cell = grid.index(*point)
    return bool(grid.inside(*cell) and wall_distance[cell] <= margin)


def surface_cells(grid, pose, marked=None, targets=None):
    """Separate occupied islands inside explored free space from its exterior."""
    data = grid.cells
    free = (data >= 0) & (data <= 25)
    regions, _ = ndimage.label(free)
    start = grid.index(*pose[:2])
    if not grid.inside(*start) or not regions[start]:
        return {}
    floor = regions == regions[start]
    interior = ndimage.binary_fill_holes(floor) & ~floor
    occupied, _ = ndimage.label(data >= 65, structure=np.ones((3, 3)))
    boxes = ndimage.find_objects(occupied)
    partial = partly_mapped_objects(occupied, boxes, floor, grid.resolution)
    if marked is not None:
        marked.update(grid, occupied, boxes, interior, partial, targets or {})
    # A wall occupies many cells. Bin samples in metres, independently on each side.
    samples = {}
    for side, (dy, dx) in enumerate(DIRECTIONS):
        adjacent = ndimage.shift(floor, (-dy, -dx), order=0, mode='constant', prefilter=False)
        rows, cols = np.where((occupied > 0) & adjacent)
        for row, col in zip(rows, cols):
            component = occupied[row, col]
            rr, cc = boxes[component-1]
            extent = max(rr.stop-rr.start, cc.stop-cc.start)*grid.resolution
            inner = bool(interior[row, col]) or component in partial
            if extent < (.30 if inner else .75):
                continue
            kind = 'interior' if inner else 'wall'
            stride = max(1, round((.45 if inner else .75)/grid.resolution))
            key = (kind, component, side, row//stride, col//stride)
            samples.setdefault(key, []).append((row, col))
    return samples


class CenterWall:
    def __init__(self):
        self.phase = 'interior'
        self.reason = 'mission_start'
        self.idle_since = None
        self.wall_anchor = None
        self.history = deque(maxlen=200)
        self.tracked_faces = TrackedFaces()
        self.batch = BoxBatch()
        self.continuity = BoxContinuity()
        self.work = FaceWork()
        self.interior_priority = InteriorPriority()
        # .51: about four minutes for one lap (the .50 Gazebo lap needed 3.2-3.3 min).
        # .57: 150 s lap. With the wall look (.55/.56) wall prints were registered at
        # 88-160 s in most Gazebo runs while laps ran to 240 s, delaying box search.
        self.wall_lap = WallLap(budget_fraction=.25, maximum_seconds=150.)
        self.chair_surface_contains = lambda point: False
        self.region_transit = RegionTransit()
        self.exploration_routes = None
        self.local_box_cost = None
        self.box_recovery = BoxFaceRecovery()
        self.marked_boxes = MarkedBoxes()
        self.cluster = ClusterExit()
        self.glass_wall = GlassWallMask()
        self.thin_obstacles = ThinObstacles()
        self.small_returns = SmallReturns()
        self.chair_band_contains = lambda point: False
        self.end_wall_surface = lambda point, yaw: False
        self.weak = WallBearings()
        self.survey = VisualSurvey()
        self.joint = RegionalSurface()
        self.bearing_advance = BearingAdvance()
        self.last_new_region = 0.
        self.last_wall_look_pose = None
        self.wall_camera = None
        self.weak_known = []
        self.weak_clock = 0.
        self.counts = {}

    def decide(self, grid, costmap, pose, transform, viewed, elapsed, duration, veto, choices=(), fitted=(), probe_only=False, pending_approach=None):
        self.wall_lap.expire(elapsed,duration)
        self.exploration_routes = None
        if transform is None or viewed is None or viewed.shape != (4,)+grid.cells.shape:
            return None
        free = (grid.cells >= 0) & (grid.cells <= 25)
        distance_cells = ndimage.distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1]
        reachable, distances = grid.reachable(pose[:2], .32, distance_cells=distance_cells)
        nav = r32.connected_nav_cells(costmap, pose)
        self.counts = {'interior': 0, 'wall': 0}
        if pending_approach is not None and not probe_only:
            approach, targets, clock = pending_approach
            goal = approach.routed_approach(grid, costmap, reachable, nav, distances, pose,
                                            targets, clock, duration-elapsed, veto)
            if goal is not None:
                self.reason = 'route_to_confirmed_print_side'
                return goal
        targets = pending_approach[1] if pending_approach else {}
        self.marked_boxes = MarkedBoxes()
        choices = tuple(choice for choice in choices
                        if not self.glass_wall.includes((choice['x'], choice['y']), .8)
                        and not self.chair_band_contains((choice['x'], choice['y'])))
        marker_positions = tuple(targets.values())
        if marker_positions:
            adjusted = []
            for choice in choices:
                factor = exploration_gain((choice['x'], choice['y']), marker_positions)
                if factor < 1.:
                    choice = dict(choice)
                    choice['features'] = np.asarray(choice['features'], dtype=float).copy()
                    choice['features'][:2] *= factor
                    choice['marker_gain'] = factor
                adjusted.append(choice)
            choices = tuple(adjusted)
        samples = surface_cells(grid, pose, self.marked_boxes, targets)
        # Both object-completion consumers use the same safely attributed IDs.
        robot_odom = odom_point(pose[:2],transform)
        self.batch.update(self.batch.centers,
            [odom_point(point,transform) for point in self.marked_boxes.markers.values()],robot_odom)
        samples = self.glass_wall.filter_samples(samples, grid)
        self.small_returns.update(grid)
        unfiltered_count = len(samples)
        samples = {key: cells for key, cells in samples.items()
                   if cells and not all(self.small_returns.cells[cell] for cell in cells)}
        small_groups_skipped = unfiltered_count-len(samples)
        self.thin_obstacles.update(grid)
        thin_skipped = 0
        clearance = distance_cells * grid.resolution
        box_safe, box_distances = box_routes(grid, costmap, nav, reachable, pose, transform, self.local_box_cost,
                                               departure=np.isfinite(distances))
        self.exploration_routes = box_distances
        choices = tuple(goal for goal in choices if route_connected(grid, box_distances, goal))
        candidates = {'interior': [], 'wall': []}
        self.tracked_faces.update(fitted, transform, elapsed)
        start = grid.index(*pose[:2])
        if not grid.inside(*start) or not np.isfinite(box_distances[start]):
            fitted = ()
        for center, faces, seen_angles in fitted:
            if self.thin_obstacles.near(center):
                thin_skipped += len(faces)
                continue
            if self.marked_boxes.fitted_done(center, faces):
                self.marked_boxes.skipped += len(faces)
                continue
            for side, (face, normal) in enumerate(faces):
                normal_yaw = math.atan2(normal[1], normal[0]) + transform[2]
                if any(math.cos(normal_yaw - angle) >= math.cos(math.radians(35)) for angle in seen_angles):
                    self.work.row(odom_point(face,transform),normal_yaw)['status']='OBSERVED'
                    continue
                anchor = odom_point(face, transform)
                self.probe_face(grid, box_distances, face, anchor, normal_yaw)
                attempts = self.attempts('interior', anchor, normal_yaw, .50)
                if not self.work.allowed(anchor,normal_yaw,elapsed):
                    continue
                tangent = np.array([-normal[1], normal[0]])
                half_width = max(abs((p-center) @ tangent) for p, _ in faces)
                fields = dict(r32_kind='interior_surface', box_view_scope='fitted_face',
                              fitted_face=side+1, face_inspection=True, stationary=False, scan_angle=math.radians(24),
                              normal_odom=normal_yaw, surface_odom=anchor.tolist(), surface_map=face.tolist(),
                              features=np.zeros(5), previous_attempts=len(attempts))
                fields['recovery_geometry'] = (anchor.tolist(), normal_yaw, float(half_width), .12)
                view = inspection_pose(grid, costmap, nav, box_distances, clearance, face, normal,
                                       half_width, pose, fields, veto, visibility_offset=.12, allow_side=True)
                if view is None and not attempts:
                    view = partial_face_pose(grid, costmap, nav, box_distances, clearance, face, normal,
                                             half_width, pose, fields, veto, max_walk=2.7)
                row=self.work.row(anchor,normal_yaw)
                row['status']='PENDING' if view else 'BLOCKED'
                row['reason']='' if view else 'no_safe_visible_viewpoint'
                if view:
                    candidates['interior'].append(view)
        # Compare generic and fitted faces together, including completed boxes.
        wall_normals = None if probe_only or self.survey.camera is None else self.survey.normals.update(grid.cells,grid.resolution)
        wall_limit = self.survey.coverage_limit()
        standoffs = self.survey.standoffs()
        wall_distance = wall_sized_distance(grid)
        on_wall_skipped = 0
        for (kind, component, side, _, _), cells in samples.items():
            if kind == 'interior' and component in self.marked_boxes.by_component:
                self.marked_boxes.skipped += 1
                continue
            if probe_only and kind != 'interior':
                continue
            # Keep one-off scan speckles out of each inspection target.
            if len(cells)*grid.resolution < .15:
                continue
            remaining = [(r,c) for r,c in cells if not viewed[side,r,c]]
            if len(remaining)*grid.resolution < .15:
                continue
            row,col = remaining[len(remaining)//2]
            if kind == 'interior' and self.thin_obstacles.near(grid.world(row, col), 0.):
                thin_skipped += 1
                continue
            # .63: a generic interior sample on a measured wall is wall surface, not a box face.
            # Fitted box faces are exempt, so a target box standing against a wall stays a task.
            if kind == 'interior' and on_wall(grid, wall_distance, grid.world(row, col)):
                on_wall_skipped += 1
                continue
            dy,dx = DIRECTIONS[side]
            face = grid.world(row+.5*dy, col+.5*dx)
            normal = (grid.world(row+dy,col+dx)-grid.world(row,col))/grid.resolution
            if kind == 'wall' and wall_normals is not None:
                normal = self.survey.wall_normal(wall_normals,row,col,dx,dy,grid.origin[2])
            normal_yaw = math.atan2(normal[1],normal[0])+transform[2]
            anchor = odom_point(face,transform)
            radius = .5 if kind == 'interior' else .85
            history_point, history_yaw = anchor, normal_yaw
            if kind == 'interior' and self.tracked_faces.boxes:
                sample = self.tracked_faces.representative(grid, remaining, dy, dx, transform, normal_yaw)
                if sample is None:
                    continue
                face, anchor, owner_face = sample
                history_point = owner_face.anchor if owner_face is not None else anchor
                history_yaw = owner_face.yaw if owner_face is not None else normal_yaw
            self.probe_face(grid, box_distances, face, history_point, history_yaw)
            attempts = self.attempts(kind, history_point, history_yaw, radius)
            if kind == 'wall':
                deferred = attempts and elapsed-attempts[-1] < min(120., 45.*len(attempts))
            else:
                deferred = not self.work.allowed(history_point,history_yaw,elapsed)
            if deferred:
                continue
            if kind == 'interior':
                fields = dict(r32_kind='interior_surface', surface_odom=history_point.tolist(),
                              normal_odom=history_yaw, surface_map=face.tolist(), previous_attempts=len(attempts),
                              features=np.zeros(5), stationary=False, face_inspection=True, scan_angle=math.radians(36))
                width = .5*len(cells)*grid.resolution
                fields['recovery_geometry'] = (odom_point(face, transform).tolist(), normal_yaw, width, .06)
                view = inspection_pose(grid, costmap, nav, box_distances, clearance, face, normal,
                                       width, pose, fields, veto, allow_side=True)
                row=self.work.row(history_point,history_yaw)
                row['status']='PENDING' if view else 'BLOCKED'
                row['reason']='' if view else 'no_safe_visible_viewpoint'
                if view:
                    candidates[kind].append(view)
                continue
            tangent = np.array([-normal[1],normal[0]])
            positions = []
            reference = standoffs[-1]
            anchor_cost = 0.
            if self.wall_anchor is not None:
                last, last_normal = self.wall_anchor
                step = anchor-last
                forward = np.array([-math.sin(last_normal), math.cos(last_normal)])
                anchor_cost = .55*np.linalg.norm(step) + 2.5*max(0.,-float(step@forward))
            for stand in standoffs:
                for shift in (0., -.28, .28):
                    point = face + normal*stand + tangent*shift
                    cell, nc = grid.index(*point), costmap.index(*point)
                    if (not grid.inside(*cell) or not reachable[cell] or not np.isfinite(distances[cell])
                            or not costmap.inside(*nc) or not nav[nc]):
                        continue
                    walk = float(distances[cell])
                    score = walk + .65*abs(stand-reference) + .3*abs(shift) + 2.*len(attempts) + anchor_cost
                    distance = math.hypot(stand, shift)
                    coverable = bool(inside_coverage_range(distance, stand / distance, wall_limit))
                    positions.append(((not coverable, score), point, walk))
            # Only the best admissible endpoint for this wall is consumed.
            # Check legal endpoints in stable cost order, stopping at the first
            # visible non-vetoed one. The safety masks above are still mandatory.
            for (outside, score), point, walk in sorted(positions, key=lambda row: row[0]):
                if not grid.line_clear(point, face+.06*normal):
                    continue
                sweep = math.radians(36)
                goal = dict(x=float(point[0]), y=float(point[1]), scan_angle=sweep,
                            yaw=math.atan2(face[1]-point[1], face[0]-point[0])-sweep/2,
                            face_inspection=True, stationary=False, features=np.zeros(5),
                            distance=float(np.linalg.norm(point-pose[:2])), r32_kind='wall_surface',
                            wall_path_m=walk, surface_odom=anchor.tolist(), normal_odom=normal_yaw,
                            surface_map=face.tolist(), previous_attempts=len(attempts),
                            wall_coverage_possible=not outside)
                if not veto(goal):
                    candidates[kind].append((score, goal))
                    break
        marker_cooled = 0
        if marker_positions:
            for category, views in candidates.items():
                adjusted = []
                for cost, goal in views:
                    factor = exploration_gain(goal['surface_map'], marker_positions)
                    marker_cooled += factor < 1.
                    adjusted.append((cost, dict(goal, marker_gain=factor)))
                candidates[category] = adjusted
        weak_walls = candidates['wall']
        ordinary_walls = [item for item in weak_walls if not self.end_wall_surface(
            item[1]['surface_map'], item[1]['normal_odom']-transform[2])]
        candidates['wall'] = [item for item in ordinary_walls if item[1]['wall_coverage_possible']]
        self.counts = {k:len(v) for k,v in candidates.items()}
        self.counts['glass_wall_skipped'] = self.glass_wall.skipped_groups
        self.counts['automatic_end_wall_skipped'] = len(weak_walls)-len(ordinary_walls)
        self.counts['marker_cooled'] = marker_cooled
        self.counts['thin_surface_skipped'] = thin_skipped
        self.counts['on_wall_skipped'] = on_wall_skipped
        self.counts['small_return_groups_skipped'] = small_groups_skipped
        self.counts['small_return_cells'] = int(self.small_returns.cells.sum())
        ordered = sorted(candidates['interior'], key=self.batch.rank)
        before_chair_strip = len(ordered)
        ordered = [item for item in ordered if not self.glass_wall.skip_generic(item[1])
                   and not self.chair_surface_contains(item[1]['surface_map'])]
        self.counts['glass_chair_skipped'] = before_chair_strip - len(ordered)
        self.counts['interior'] = len(ordered)
        for _, goal in ordered:
            if goal.get('batch_box') is not None:
                goal['recovery_view_odom'] = odom_point((goal['x'], goal['y']), transform).tolist()
        robot = odom_point(pose[:2], transform)
        mode = None if probe_only else self.region_transit.mode(elapsed, robot, self.survey.trail)
        if not probe_only:
            # Preserve the complete existing wall candidate set for measured
            # marker bearings, including surfaces normally omitted from coverage.
            self.survey.progress(samples, grid, viewed, transform, pose, elapsed, DIRECTIONS)
            clue = self.bearing_advance.choose(grid, costmap, reachable, nav, distances, pose, transform,
                self.survey.known, self.weak_clock, duration-elapsed, veto,
                expected_count=pending_approach[0].expected_marker_count if pending_approach else 0)
            if clue is None:
                clue = self.weak.choose(weak_walls, grid, transform, self.weak_known,
                    self.weak_clock, duration-elapsed, self.survey.camera, self.survey.reference_m)
            if clue is None:
                proposal = self.survey.choose(grid, costmap, nav, reachable, distances, pose, transform,
                    candidates['wall'], ordered, elapsed, duration, veto)
                clue = proposal if hinted(proposal) else None
            if clue is not None:
                self.phase = 'wall' if clue.get('weak_wall') else 'interior'
                self.reason = 'wall_marker_evidence'
                return clue
            if self.wall_lap.phase == 'perimeter':
                lap = self.wall_lap.choose(grid, box_distances, pose, transform, elapsed, duration, veto)
                if lap is not None:
                    self.phase,self.reason='wall','quick_perimeter_pass'
                    return lap
                if self.wall_lap.phase == 'perimeter':
                    self.reason='waiting_for_safe_perimeter'
                    return None
        # Region resume/escape and object service share this single arbitration.
        crowded = self.cluster.crowded_centers(elapsed)
        ranked = [(walk,g) for walk,g in ordered
                  if not g.get('completed_box') and self.continuity.available(g,elapsed)
                  and not self.continuity.recently_swept(g,elapsed)]
        ranked = self.continuity.unswept_first(ranked)
        if crowded:
            ranked = [(walk,g) for walk,g in ranked if not self.cluster.local_fragment(g,crowded)]
        if probe_only:
            return min(ranked,key=lambda row:row[0])[1] if ranked else None
        active_service = self.continuity.box is not None and elapsed-self.continuity.entered < RULES.box_visit_s
        if mode and (mode == 'resume' or not active_service):
            transit = self.region_transit.choose(mode,choices,grid,box_distances,transform,
                robot,self.survey.visited_positions(elapsed),duration-elapsed,veto)
            if transit is not None:
                self.phase,self.reason = 'interior','region_'+mode
                return transit
        resumed = self.continuity.choose(ranked,elapsed,duration-elapsed,grid,box_safe)
        if resumed is not None:
            self.phase='interior'
            self.reason='resume_interrupted_face' if resumed.get('box_face_resume') else 'finish_current_box'
            return resumed
        recovery = self.box_recovery.choose(grid,costmap,box_safe,nav,box_distances,clearance,
            pose,transform,veto,ranked,elapsed,duration,self.glass_wall.in_zone)
        if recovery is not None and self.continuity.available(recovery,elapsed):
            self.phase,self.reason='interior','box_safe_retry'
            return recovery
        frontier = self.interior_priority.choose(grid,box_distances,transform,(),choices,
            weak_walls,self.survey.visited_positions(elapsed),duration-elapsed,veto)
        # Joint NBV keeps incidental wall gains without restarting ordinary wall service.
        combined = self.joint.choose(grid,box_safe,box_distances,transform,self.survey,
            candidates['wall'],ranked,None,elapsed,duration,frontier=frontier,
            heading=pose[2],allow_wall_destinations=False)
        if combined is None:
            combined = self.interior_priority.choose(grid,box_distances,transform,ranked,choices,
                weak_walls,self.survey.visited_positions(elapsed),duration-elapsed,veto)
        if combined is not None:
            self.phase='interior'
            self.reason=('interior_first_exploration' if combined.get('r32_kind')=='interior_frontier'
                         else 'regional_camera_gain_per_route')
            return combined
        self.reason='interior_tasks_temporarily_unavailable'
        return None

    def probe_face(self, grid, distances, face, anchor, yaw):
        # Fixed physical samples are independent of map-array resizing.
        values=[]
        for dy in (-.9,-.6,-.3,0.,.3,.6,.9):
            for dx in (-.9,-.6,-.3,0.,.3,.6,.9):
                cell=grid.index(face[0]+dx,face[1]+dy)
                if not grid.inside(*cell):
                    values.append(-2)
                else:
                    v=grid.cells[cell]
                    values.append(-1 if v<0 else 2 if v>25 else 1 if np.isfinite(distances[cell]) else 0)
        return self.work.probe(anchor,yaw,values)

    def observe_wall_here(self, goal, grid, costmap, nav, reachable, pose, transform, wall_limit, veto):
        """Replace a trip to a single wall face only if this pose already qualifies."""
        if (goal['r32_kind'] != 'wall_surface' or goal.get('joint_views') != 1 or
                (goal.get('survey_visible_sections') or 0) > 1 or goal.get('previous_attempts')):
            return goal
        xy = np.asarray(pose[:2])
        here = odom_point(xy, transform)
        if self.last_wall_look_pose is not None and math.dist(here, self.last_wall_look_pose) < .32:
            return goal
        cell, nc = grid.index(*xy), costmap.index(*xy)
        if not grid.inside(*cell) or not reachable[cell] or not costmap.inside(*nc) or not nav[nc]:
            return goal
        face = np.asarray(goal['surface_map'])
        normal_yaw = goal['normal_odom'] - transform[2]
        normal = np.array([math.cos(normal_yaw), math.sin(normal_yaw)])
        if self.wall_camera is None or self.survey.camera is None:
            return goal
        optical = np.array(self.wall_camera[:2])
        heading = self.wall_camera[2]
        aim = math.atan2(face[1]-optical[1], face[0]-optical[0])
        sweep = goal['scan_angle']
        if abs(math.remainder(aim+sweep/2-heading, math.tau)) > 2*math.pi/3:
            return goal
        # Account for the optical centre sweeping around the base during a turn.
        offset = optical - xy
        for look_yaw in (aim-sweep/2, aim, aim+sweep/2):
            rotation = look_yaw-heading
            c, s = math.cos(rotation), math.sin(rotation)
            eye = xy + np.array([c*offset[0]-s*offset[1], s*offset[0]+c*offset[1]])
            ray = face - eye
            distance = float(np.linalg.norm(ray))
            if not distance or not inside_coverage_range(distance, -float(ray @ normal)/distance, wall_limit):
                return goal
            if abs(math.remainder(math.atan2(ray[1], ray[0])-look_yaw, math.tau)) >= self.survey.camera[1]/2:
                return goal
            if not grid.line_clear(eye, face + .06*normal):
                return goal
        look = dict(goal, stationary=True, x=float(xy[0]), y=float(xy[1]), yaw=aim-sweep/2,
                    distance=0., joint_walk_m=0., original_wall_route_m=goal['joint_walk_m'],
                    current_wall_pose_odom=here.tolist())
        return goal if veto(look) else look

    def face_result(self, kind, status, reason, elapsed):
        self.work.result(kind,status,reason,elapsed)
        undone = self.continuity.result(kind, status, reason, elapsed)
        if undone is None:
            return
        # Only undo the exact dispatch that was interrupted before its sweep.
        anchor = np.asarray(undone['surface_odom'])
        stamp = undone['face_dispatch_elapsed']
        self.history = deque((r for r in self.history if not (
            r[0] == 'interior' and r[3] == stamp and np.linalg.norm(r[1]-anchor) < .01)), maxlen=200)
        key = undone.get('region_face')
        if key in self.joint.attempted:
            self.joint.attempted.discard(key)
            self.joint.stops = max(0, self.joint.stops-1)

    def attempts(self, kind, anchor, normal_yaw, radius):
        if kind == 'interior':
            return self.work.attempts(anchor,normal_yaw)
        return [t for k,p,a,t in self.history if k == kind and
                np.linalg.norm(anchor-p)<radius and math.cos(normal_yaw-a)>.70]

    def dispatched(self, goal, elapsed):
        self.work.dispatched(goal)
        self.wall_lap.dispatched(goal)
        self.continuity.dispatched(dict(goal, face_dispatch_elapsed=elapsed), elapsed)
        self.region_transit.dispatched(goal, elapsed, self.joint.robot_xy)
        if 'current_wall_pose_odom' in goal:
            self.last_wall_look_pose = tuple(goal['current_wall_pose_odom'])
        self.on_weak_route = bool(goal.get('weak_wall'))
        self.weak.dispatched(goal)
        self.bearing_advance.dispatched(goal)
        self.survey.dispatched(goal, elapsed)
        self.on_cluster_exit = bool(goal.get('cluster_exit'))
        self.cluster.dispatched(goal,elapsed)
        self.batch.dispatched(goal)
        self.box_recovery.record_pick(goal, elapsed)
        self.box_recovery.dispatched(goal)
        self.joint.dispatched(goal, elapsed)
        if goal.get('survey_region'):
            self.last_new_region = elapsed
        if 'surface_odom' in goal:
            anchor = np.asarray(goal['surface_odom'])
            normal = goal['normal_odom']
            kind = 'wall' if goal.get('r32_kind') == 'wall_surface' else 'interior'
            self.history.append((kind,anchor,normal,elapsed))
            if self.phase=='wall':
                self.wall_anchor = anchor,normal
