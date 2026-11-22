"""Select safe views for persistent prints, overlooked wall sections and low-gain loops."""
from collections import deque
import math

import numpy as np

from osp_mission.normal_cache import AxisNormalCache


def rotation(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]])


def odom(point, transform):
    return rotation(transform[2]) @ point + transform[:2]


def from_odom(point, transform):
    return rotation(transform[2]).T @ (np.asarray(point) - transform[:2])


def safe_goal(grid, cost, nav, safe, distances, point, face, normal, veto, goal):
    cell = grid.index(*point)
    nc = cost.index(*point)
    if (not grid.inside(*cell) or not safe[cell] or not np.isfinite(distances[cell]) or
            not cost.inside(*nc) or not nav[nc] or not grid.line_clear(point, face + .10*normal)):
        return None
    distance = float(distances[cell])
    yaw = math.atan2(face[1]-point[1], face[0]-point[0])
    item = dict(goal, x=float(point[0]), y=float(point[1]), distance=distance, stationary=False,
                yaw=yaw-.5*goal['scan_angle'], features=np.array([0., 0., distance, 0., 0.]))
    return None if veto(item) else item


class VisualSurvey:
    def __init__(self):
        self.trail = deque(maxlen=480)
        self.normals = AxisNormalCache()
        self.evidence = set()
        self.last_progress = -math.inf
        self.wall_times = {}
        self.last_wall = 0.
        self.last_direction = {}
        self.direction_count = {}
        self.direction_sections = {}
        self.confirmed_sections = {}
        self.last_camera_wall = {}
        self.hints = {}
        self.hint_attempts = {}
        self.known = set()
        self.loop_center = None
        self.loop_until = -math.inf
        self.tried_new_face = False
        self.last_escape = -math.inf
        self.last_was_wall = False
        self.last_region = 0.
        self.camera = None
        self.reference_m = .14
        self.readable_px = 18.

    def standoffs(self):
        if not self.camera or self.reference_m <= 0 or self.readable_px <= 0:
            return (.67,.88,1.15)
        distance = float(np.clip(.85*self.camera[0]*self.reference_m/self.readable_px,1.25,3.))
        return (.67,.88,1.15,.75*distance,distance)

    @staticmethod
    def wall_normal(fields, row, col, dx, dy, yaw):
        dy_field, dx_field = fields
        direction = np.array([dx_field[row,col],dy_field[row,col]])
        norm = np.linalg.norm(direction)
        alignment = direction @ [dx,dy]
        if norm < 1.e-8 or abs(alignment) < .25*norm:
            direction = np.array([dx,dy],dtype=float)
        else:
            direction *= (1. if alignment > 0 else -1.)/norm
        return rotation(yaw) @ direction

    def position(self, x, y, t):
        if not all(map(math.isfinite, (x, y, t))):
            return
        if self.trail:
            last = self.trail[-1]
            step = math.hypot(x-last[1], y-last[2])
            if t < last[0] or step > 1.5:
                self.trail.clear()
                self.loop_center = None
            elif step < .22:
                return
        self.trail.append((t, x, y))

    def printed(self, event, clock, elapsed):
        try:
            if event.get('source') != 'distant_native_multiview' or not -.1 <= clock-event['stamp'] <= .75:
                return False
            ident = int(event['id'])
            point, observer = (np.asarray(event[key], dtype=float)[:2] for key in ('point_odom','observer_odom'))
            if (not 0 <= ident < 1024 or point.shape!=(2,) or observer.shape!=(2,) or
                    not np.isfinite([*point,*observer]).all() or not .50 <= np.linalg.norm(point-observer) <= 5.5):
                return False
        except (KeyError, TypeError, ValueError, IndexError, OverflowError):
            return False
        if ident in self.known or (ident in self.hints and event['stamp'] <= self.hints[ident]['stamp']):
            return False
        self.hints = {i:h for i,h in self.hints.items() if elapsed-h['elapsed'] <= 65.}
        self.hints[ident] = dict(point=point, observer=observer, stamp=event['stamp'], elapsed=elapsed)
        return True

    def progress(self, samples, grid, viewed, transform, pose, elapsed, directions):
        """Count separated, camera-confirmed face samples near the current route in odom."""
        origin = odom(grid.origin[:2], transform)
        a = grid.origin[2]+transform[2]
        axes = rotation(a)
        start = grid.index(*pose[:2])
        new = set()
        wall_offsets = {}
        for (kind, _, side, _, _), cells in samples.items():
            rc = np.asarray(cells)
            if np.linalg.norm(rc[0]-start)*grid.resolution > 4.5:
                continue
            rc = rc[viewed[side, rc[:,0], rc[:,1]]]
            if not len(rc):
                continue
            dy, dx = directions[side]
            normal = axes @ [dx, dy]
            angle_bin = round(math.atan2(normal[1],normal[0])/(math.pi/4)) % 8
            xy = ((rc[:, ::-1]+.5+.5*np.array([dx,dy]))*grid.resolution) @ axes.T + origin
            bins = np.floor(xy/.35).astype(int)
            direction = self.cardinal_wall({'normal_odom': math.atan2(normal[1],normal[0])})
            for point,(x,y) in zip(xy,bins):
                item=(int(x),int(y),angle_bin)
                if kind == 'wall' and item not in self.evidence and item not in new:
                    wall_offsets.setdefault(direction,[]).append(
                        self.wall_offset({'surface_odom': point},direction))
                new.add(item)
        if len(new-self.evidence) >= 2:
            self.last_progress = elapsed
            self.loop_center = None
        self.evidence.update(new)
        # Count a camera-confirmed wall zone even when the robot turned to
        # inspect a box. One scan contributes at most one zone per wall side.
        for direction, offsets in wall_offsets.items():
            if elapsed-self.last_camera_wall.get(direction,-math.inf)<5.:
                continue
            offset=float(np.median(offsets))
            positions=self.confirmed_sections.setdefault(direction,[])
            if all(abs(offset-old)>=1.25 for old in positions):
                positions.append(offset)
                self.direction_count[direction]=len(positions)
                self.last_camera_wall[direction]=elapsed

    def repeated_route(self, elapsed):
        if elapsed-self.last_progress < 20. or elapsed-self.last_escape < 35.:
            return False
        trail = np.array([p for p in self.trail if 0 <= elapsed-p[0] <= 85.])
        if len(trail) < 12:
            return False
        recent = trail[-4:,1:]
        old = trail[trail[:,0] < trail[-4,0]-14.]
        if len(old) < 5:
            return False
        # At least three successive odometry samples retrace an old corridor,
        # after having actually left it. A single crossing cannot qualify.
        near = np.linalg.norm(recent[:,None,:]-old[None,:,1:], axis=2) <= .48
        if near.any(axis=1).sum() < 3:
            return False
        previous = old[np.flatnonzero(near.any(axis=0))[-1],0]
        excursion = trail[trail[:,0] >= previous,1:]
        return (np.linalg.norm(np.diff(excursion,axis=0),axis=1).sum() >= 3. and
                np.max(np.linalg.norm(excursion-recent[-1],axis=1)) >= 1.05)

    def wall_key(self, goal):
        normal = goal['normal_odom']
        side = round(normal/(math.pi/4)) % 8
        axis = np.array([math.cos(side*math.pi/4),math.sin(side*math.pi/4)])
        point = goal['surface_odom']
        return side, round(axis@point/.65), math.floor(np.array([-axis[1],axis[0]])@point/1.6)

    @staticmethod
    def cardinal_wall(goal):
        """Group outward wall normals into four world-fixed directions."""
        return math.floor((goal['normal_odom'] + math.pi / 4) / (math.pi / 2)) % 4

    @staticmethod
    def wall_offset(goal, direction):
        theta = direction * math.pi / 2
        point = goal['surface_odom']
        return -math.sin(theta) * point[0] + math.cos(theta) * point[1]

    def region_goal(self, choices, grid, cost, nav, safe, distances, transform,
                    elapsed, duration, veto):
        """Give undiscovered regions a turn even while local object views remain."""
        if elapsed < 120. or elapsed-self.last_region < 100. or duration-elapsed < 55.:
            return None
        recent = np.array([p[1:] for p in self.trail if elapsed-p[0] <= 170.])
        options = []
        for candidate in choices:
            features = candidate['features']
            if (len(features) < 5 or (features[1] < .15 and features[0] < .35)
                    or candidate.get('stationary') or veto(candidate)):
                continue
            point = candidate['x'],candidate['y']
            cell, nc = grid.index(*point), cost.index(*point)
            if (not grid.inside(*cell) or not safe[cell] or not np.isfinite(distances[cell])
                    or not cost.inside(*nc) or not nav[nc]):
                continue
            walk = float(distances[cell])
            if walk/.20+5. > duration-elapsed:
                continue
            odom_point = odom(point,transform)
            novelty = float(np.linalg.norm(recent-odom_point,axis=1).min()) if len(recent) else 4.
            if novelty < 1.5:
                continue
            # Edge planner: [unseen surface, frontier gain, travel, turn, revisit].
            # A mapped wall can still be unseen by the camera when frontier gain is zero.
            score = .5*features[0] + .3*features[1] + .7*min(novelty,8.) - .03*walk
            options.append((score,candidate))
        if not options:
            return None
        return dict(max(options,key=lambda item:item[0])[1],r32_kind='interior_frontier',survey_region=True)

    def choose(self, grid, cost, nav, safe, distances, pose, transform, walls, interiors,
               elapsed, duration, veto):
        if self.repeated_route(elapsed) and (self.loop_center is None or elapsed > self.loop_until):
            self.loop_center = odom(pose[:2],transform)
            self.loop_until = elapsed+45.
            self.tried_new_face = False
        if elapsed > self.loop_until:
            self.loop_center = None
        if len(self.hint_attempts) < 3 and duration-elapsed >= 10.:
            for ident,h in sorted(self.hints.items(), key=lambda item:-item[1]['elapsed']):
                if ident in self.known or ident in self.hint_attempts or not 0 <= elapsed-h['elapsed'] <= 65.:
                    continue
                face = from_odom(h['point'],transform)
                normal = from_odom(h['observer'],transform)-face
                normal /= np.linalg.norm(normal)
                if (np.asarray(pose[:2])-face)@normal <= 0:
                    continue
                tangent = np.array([-normal[1], normal[0]])
                options = []
                for stand in (1.15,1.55,2.1,2.7):
                    for lateral in (0.,-.40,.40):
                        point=face+stand*normal+lateral*tangent
                        if np.linalg.norm(point-face) > np.linalg.norm(np.asarray(pose[:2])-face)-.30:
                            continue
                        goal = safe_goal(grid,cost,nav,safe,distances,point,face,normal,veto,
                                         dict(r32_kind='distant_print',distant_hint_id=ident,
                                              face_inspection=True,scan_angle=0.))
                        if goal is not None and goal['distance']/.20+4 <= min(36.,duration-elapsed):
                            options.append((goal['distance']/.25+3.*stand,goal))
                if options:
                    return min(options,key=lambda pair:pair[0])[1]
        if self.loop_center is not None and not self.tried_new_face:
            for _,goal in interiors:
                if ('fitted_face' in goal and not goal.get('previous_attempts') and
                        np.linalg.norm(goal['surface_odom']-self.loop_center) < 2.2):
                    return dict(goal, survey_new_face=True)
        if self.camera is None or not walls:
            return None
        fx, hfov = self.camera
        if not (fx > 0 and 0 < hfov < math.pi and self.readable_px > 0 and self.reference_m > 0):
            return None
        period = min(65.,max(20.,duration*.10))
        entries = []
        for _,g in walls:
            key = self.wall_key(g)
            direction = self.cardinal_wall(g)
            since, last = self.wall_times.setdefault(key,(elapsed,-math.inf))
            if elapsed-last < period or (self.loop_center is not None and
                    np.linalg.norm(g['surface_odom']-self.loop_center) < 2.2):
                continue
            walk = distances[grid.index(g['x'],g['y'])]
            age = elapsed-max(since,last)
            direction_age = elapsed-self.last_direction.get(direction,0.)
            entries.append((min(2.,age/period)+min(2.,direction_age/period)-.085*walk,g,key,direction))
        if not entries or (self.loop_center is None and
                (elapsed-self.last_wall < period or (interiors and self.last_was_wall))):
            return None
        entries.sort(key=lambda x:-x[0])
        # Include both ends of a long wall as well as its best nearby section.
        # Nearest-first sampling alone can omit the far end indefinitely.
        by_direction = {}
        for item in entries:
            by_direction.setdefault(item[3], []).append(item)
        selected = []
        selected_ids = set()
        for direction, group in by_direction.items():
            key = lambda item: self.wall_offset(item[1], direction)
            choices = (min(group,key=key), max(group,key=key),
                       max(group,key=lambda item:item[1].get('unseen_wall_m',0.)/
                           (1.+.15*item[1]['distance'])))
            for item in choices:
                identity = id(item[1])
                if identity not in selected_ids:
                    selected.append(item)
                    selected_ids.add(identity)
        all_faces = np.array([g['surface_map'] for _,g in walls])
        all_normals = np.array([g['normal_odom']-transform[2] for _,g in walls])
        units = np.stack((np.cos(all_normals),np.sin(all_normals)), axis=1)
        reference = self.standoffs()[-1]
        options = []
        for age,g,key,direction in selected:
            face = np.array(g['surface_map'])
            normal = np.array([math.cos(g['normal_odom']-transform[2]),math.sin(g['normal_odom']-transform[2])])
            tangent = np.array([-normal[1],normal[0]])
            points = [face+stand*normal+shift*tangent for stand in (reference,.75*reference)
                      for shift in (0.,-.4,.4)]
            points.append(np.array([g['x'],g['y']]))
            for point in points:
                goal = safe_goal(grid,cost,nav,safe,distances,point,face,normal,veto,
                                 dict(g,scan_angle=math.radians(22),
                                      survey_wall=list(key)))
                if goal is None or goal['distance']/.20+4 > duration-elapsed:
                    continue
                delta = all_faces-point
                ranges = np.maximum(np.linalg.norm(delta,axis=1),.01)
                aligned = -(units*delta).sum(axis=1)/ranges
                yaw = goal['yaw']+.5*goal['scan_angle']
                visible = (delta@np.array([math.cos(yaw),math.sin(yaw)])/ranges >= math.cos(.44*hfov))
                readable = (aligned >= .55) & (fx*self.reference_m*aligned/ranges >= self.readable_px)
                ids = np.flatnonzero(visible & readable)
                # Count distinct, actually unobserved wall sections in line of sight.
                seen = {}
                for i in ids[np.argsort(ranges[ids])][:12]:
                    other=walls[i][1]
                    section = self.wall_key(other)
                    if section not in seen and grid.line_clear(point,all_faces[i]+.10*units[i]):
                        seen[section] = other.get('unseen_wall_m',0.)
                if not seen:
                    continue
                gain = min(3.,sum(seen.values()))
                if gain < .15:
                    continue
                stand=float(np.linalg.norm(point-face))
                offset=self.wall_offset(g,direction)
                previous=self.direction_sections.get(direction,())
                section_novelty=min((abs(offset-old) for old in previous),default=4.)
                # Prefer a few wide views over many close wall stops. The
                # actual readable-pixel and line-of-sight checks above remain.
                score=(2.5*gain+.25*age+.15*min(4.,section_novelty))/(
                    1.+.10*goal['distance'])-.5*max(0.,1.5-stand)
                goal.update(survey_visible_sections=len(seen),
                            survey_unseen_wall_m=round(gain,2),standoff_m=stand)
                options.append((score,goal,direction))
        if not options:
            return None
        return max(options,key=lambda item:item[0])[1]

    def remote_view(self, choices, grid, cost, nav, safe, distances, transform, elapsed, remaining, veto):
        if self.loop_center is None:
            return None
        visited = np.array([p[1:] for p in self.trail if elapsed-p[0] <= 170.])
        options = []
        for candidate in choices:
            xy = (candidate['x'],candidate['y'])
            place = odom(xy,transform)
            if candidate.get('stationary') or np.linalg.norm(place-self.loop_center)<2.4:
                continue
            cell, nc = grid.index(*xy), cost.index(*xy)
            if (not grid.inside(*cell) or not safe[cell] or not cost.inside(*nc) or not nav[nc]
                    or not np.isfinite(distances[cell]) or veto(candidate)):
                continue
            walk = distances[cell]
            if walk/.20+3 > remaining:
                continue
            fresh = float(np.linalg.norm(visited-place,axis=1).min()) if len(visited) else 3.
            gain = candidate['features'][0] + candidate['features'][1] + min(3.,fresh)
            if fresh >= .9 and gain > 0:
                options.append((gain/(1.+.2*walk),candidate))
        if options:
            return dict(max(options,key=lambda pair:pair[0])[1],r32_kind='unvisited_region')
        return None

    def dispatched(self, goal, elapsed):
        if goal.get('survey_region'):
            self.last_region = elapsed
        if goal.get('survey_new_face'):
            self.tried_new_face = True
        self.last_was_wall = bool(goal.get('survey_wall'))
        if 'survey_wall' in goal:
            key = tuple(goal['survey_wall'])
            self.wall_times[key] = self.wall_times[key][0],elapsed
            self.last_wall=elapsed
            direction=self.cardinal_wall(goal)
            self.last_direction[direction]=elapsed
            positions=self.direction_sections.setdefault(direction,[])
            offset=self.wall_offset(goal,direction)
            if all(abs(offset-old)>=1.25 for old in positions):
                positions.append(offset)
            del positions[:-20]
        if 'distant_hint_id' in goal:
            self.hint_attempts[goal['distant_hint_id']]=elapsed
        if goal.get('survey_escape'):
            self.last_escape=elapsed
