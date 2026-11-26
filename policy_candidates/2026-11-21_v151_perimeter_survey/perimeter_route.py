"""Continue a camera-observed perimeter after bounded object detours."""
from dataclasses import dataclass
import math
import numpy as np
from osp_mission.adaptive_faces import inspection_pose
from osp_mission.box_views import odom_point
from osp_mission.edge_policy import DIRECTIONS
from osp_mission.normal_cache import AxisNormalCache


@dataclass
class WallSection:
    point: np.ndarray
    normal: np.ndarray
    half_width: float
    fraction: float = 0.
    observed: bool = False
    attempts: int = 0
    failures: int = 0
    deferred_until: float = 0.
    reason: str = 'awaiting_camera'
    updated: float = 0.


class PerimeterRoute:
    def __init__(self):
        self.sections = {}
        self.normal_cache = AxisNormalCache()
        self.spacing = .90
        self.camera = None
        self.axis = None
        self.cursor = None
        self.current = None
        self.action_section = None
        self.arrived = None
        self.last_event = 'start'
        self.remaining_estimate_s = 0.

    def key(self, point, normal_yaw):
        side = round((normal_yaw-self.axis)/(math.pi/2)) % 4
        angle = self.axis+side*math.pi/2
        normal = np.array([math.cos(angle),math.sin(angle)])
        tangent = np.array([-normal[1],normal[0]])
        return side, round(float(point@normal)/.25), math.floor(float(point@tangent)/self.spacing)

    def observe(self, samples, grid, viewed, transform, owner, elapsed):
        normal_y,normal_x = self.normal_cache.update(grid.cells,grid.resolution)
        records = []
        for (kind, _, side, _, _), cells in samples.items():
            if kind != 'wall':
                continue
            dy,dx = DIRECTIONS[side]
            for row,col in cells:
                point = odom_point(grid.world(row+.5*dy,col+.5*dx),transform)
                if owner(dict(surface_odom=point.tolist()),transform) is not None:
                    continue
                nx,ny = normal_x[row,col],normal_y[row,col]
                alignment = nx*dx+ny*dy
                if abs(alignment)<.25:
                    nx,ny = dx,dy
                elif alignment<0:
                    nx,ny = -nx,-ny
                angle = math.atan2(ny,nx)+grid.origin[2]+transform[2]
                records.append((point,bool(viewed[side,row,col]),angle))
        if not records:
            return
        if self.axis is None:
            angles = np.array([a for _,_,a in records])
            self.axis = math.atan2(float(np.sin(4*angles).sum()),float(np.cos(4*angles).sum()))/4
        groups = {}
        for point,seen,angle in records:
            groups.setdefault(self.key(point,angle),[]).append((point,seen,angle))
        for key, cells in groups.items():
            if len(cells)*grid.resolution < .20:
                continue
            point = np.mean([p for p,_,_ in cells],axis=0)
            angle = math.atan2(sum(math.sin(a) for _,_,a in cells),sum(math.cos(a) for _,_,a in cells))
            normal = np.array([math.cos(angle),math.sin(angle)])
            section = self.sections.get(key)
            if section is None:
                section = WallSection(point,normal,self.spacing/2)
                self.sections[key] = section
            section.point = point
            section.normal = normal
            section.updated = elapsed
            # Merge projected witness intervals. Stair-step grid edges at a
            # slanted wall must not double-count the same physical surface.
            direction = self.axis+key[0]*math.pi/2
            tangent = np.array([-math.sin(direction),math.cos(direction)])
            lo,hi = key[2]*self.spacing,(key[2]+1)*self.spacing
            spans = sorted((max(lo,float(p@tangent)-grid.resolution/2),
                            min(hi,float(p@tangent)+grid.resolution/2))
                           for p,seen,_ in cells if seen)
            covered = 0.
            right = lo
            for left,end in spans:
                covered += max(0.,end-max(right,left))
                right = max(right,end)
            section.fraction = max(section.fraction,min(1.,covered/self.spacing))
            if section.fraction >= .75:
                section.observed = True
                section.reason = 'camera_confirmed_wall'
        if self.arrived is not None:
            section = self.sections[self.arrived]
            if not section.observed:
                section.failures += 1
                section.deferred_until = elapsed+min(120.,30.*section.failures)
                section.reason = 'post_sweep_camera_evidence_insufficient'
            self.arrived = None
        pending = sum(not s.observed for s in self.sections.values())
        # Planning estimate from sampled unseen wall length and bounded dwell.
        # Inter-section routing and obstacle detours can make actual time longer.
        self.remaining_estimate_s = pending*(self.spacing/.25+2.5)

    def local_geometry(self, section, transform):
        tx,ty,theta = transform
        c,s = math.cos(theta),math.sin(theta)
        rotation = np.array([[c,s],[-s,c]])
        return rotation@(section.point-[tx,ty]),rotation@section.normal

    def choose(self, grid, cost, pose, transform, distances, clearance, nav, veto, elapsed, duration):
        goals = []
        if self.camera is None:
            return None
        focal,_ = self.camera
        readable = .85*focal*.08/18.
        preferred = min(1.,.80*readable)
        for key, section in self.sections.items():
            if section.observed or elapsed<section.deferred_until or elapsed-section.updated>45.:
                continue
            point,normal = self.local_geometry(section,transform)
            fields = dict(r32_kind='wall_surface',surface_map=point.tolist(),
                surface_odom=section.point.tolist(),normal_odom=math.atan2(section.normal[1],section.normal[0]),
                features=np.zeros(5),scan_angle=math.radians(36),face_inspection=True,stationary=False,
                perimeter_section=list(key))
            view = inspection_pose(grid,cost,nav,distances,clearance,point,normal,
                                   section.half_width,pose,fields,veto,
                                   minimum_standoff=max(.40,.85*preferred),
                                   maximum_standoff=min(1.20,1.05*preferred),
                                   preferred_standoff=preferred)
            if view is None:
                # A narrow passage can require a closer, partial observation.
                # Completion still requires actual evidence over the full bin.
                view = inspection_pose(grid,cost,nav,distances,clearance,point,normal,
                                       min(.25,section.half_width),pose,fields,veto,
                                       preferred_standoff=preferred)
            if view is None:
                section.reason = 'no_safe_visible_connected_view'
                continue
            walk,goal = view
            if walk/.20+3.>duration-elapsed:
                section.reason = 'insufficient_remaining_time_estimate'
                continue
            continuity = 0.
            if self.cursor is not None:
                anchor,old_normal = self.cursor
                tangent = np.array([-old_normal[1],old_normal[0]])
                step = section.point-anchor
                gap = np.linalg.norm(step)
                forward = float(step@tangent)
                same_wall = float(old_normal@section.normal)>.85
                if same_wall and -.2<=forward<=2.5 and gap<=2.6:
                    continuity = 5.
                elif gap<=2.:
                    continuity = 3.
                continuity -= min(2.,gap/5.)
            reward = continuity + (3. if key==self.current else 0.) + 1.-section.fraction
            score = reward-.30*walk-.5*section.failures
            goals.append((score,goal,key))
        if not goals:
            return None
        _,goal,key = max(goals,key=lambda item:item[0])
        self.last_event = 'resume_camera_unseen_wall'
        return goal

    def dispatched(self, goal):
        self.action_section = None
        if 'perimeter_section' not in goal:
            return
        key = tuple(goal['perimeter_section'])
        section = self.sections[key]
        section.attempts += 1
        self.current = key
        self.action_section = key
        self.cursor = section.point.copy(),section.normal.copy()

    def outcome(self, status, reason, elapsed):
        key = self.action_section
        self.action_section = None
        if key is None:
            return
        section = self.sections[key]
        if status==4:
            self.arrived = key
        elif status==6 or reason in ('navigation_stalled','action_timeout','bump_contact'):
            section.failures += 1
            section.deferred_until = elapsed+min(120.,30.*section.failures)
            section.reason = reason or 'nav2_failed'

    def report(self):
        return dict(current=list(self.current) if self.current is not None else None,
            mapped_sections=len(self.sections),observed_sections=sum(s.observed for s in self.sections.values()),
            remaining_time_estimate_s=self.remaining_estimate_s,
            unfinished=[dict(id=list(k),fraction=round(s.fraction,3),reason=s.reason,attempts=s.attempts,
                             anchor_odom=s.point.tolist()) for k,s in self.sections.items() if not s.observed])
