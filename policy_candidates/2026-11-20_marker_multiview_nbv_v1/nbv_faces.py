"""Planar camera model and bounded, odom-anchored surface inspection memory."""
from dataclasses import dataclass
import math
import numpy as np
from osp_mission import _r32_navigation_core as r32
from osp_mission.adaptive_faces import clearance_grid, indices
from osp_mission.box_views import odom_point
from osp_mission.center_wall import CenterWall, surface_cells
from osp_mission.edge_policy import DIRECTIONS

def axis(angle):
    return np.array([math.cos(angle), math.sin(angle)])

def angle(v):
    return math.atan2(v[1], v[0])

def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))

@dataclass
class Face:
    point: np.ndarray
    normal: float
    best: float = 0.
    visits: int = 0
    last: float = -math.inf

class MultiView(CenterWall):
    def __init__(self):
        super().__init__()
        self.faces = []
        self.local = []
        self.camera = None  # actual focal length, FOV and base-to-camera planar extrinsics
        self.exposures = 0

    def register(self, point, normal):
        nearest = None
        distance = .26
        for i, old in enumerate(self.faces):
            delta = math.dist(point, old.point)
            if delta < distance and math.cos(normal-old.normal) > .85:
                nearest, distance = i, delta
        if nearest is None:
            nearest = len(self.faces)
            self.faces.append(Face(point, normal))
        return nearest

    def geometry(self, grid, transform, pose, fitted):
        faces = []
        for center, sides, _ in fitted:
            for p, n in sides:
                tangent = np.array([-n[1], n[0]])
                width = max(abs((q-center)@tangent) for q,_ in sides)
                faces.append((p,n,width,'interior',True))
        for (kind, _, side, _, _), cells in surface_cells(grid, pose).items():
            if len(cells)*grid.resolution < .15:
                continue
            row,col = cells[len(cells)//2]
            dr,dc = DIRECTIONS[side]
            p = grid.world(row+.5*dr,col+.5*dc)
            if math.dist(p,pose[:2]) > 5.0:
                continue
            n = (grid.world(row+dr,col+dc)-grid.world(row,col))/grid.resolution
            if any(math.dist(p,q)<width+.16 and n@normal>.7 for q,normal,width,_,box in faces if box):
                continue
            faces.append((p,n,min(.45,len(cells)*grid.resolution/2),kind,False))
        local = {}
        for p,n,width,kind,box in faces:
            key = self.register(odom_point(p,transform),angle(n)+transform[2])
            local.setdefault(key,(key,p,n,width,kind,box))
        self.local = list(local.values())
        return self.local

    @staticmethod
    def quality(grid, camera, heading, focal, fov, face):
        _,p,n,width,_,_ = face
        d = p-camera
        distance = np.linalg.norm(d)
        if not .15 < distance <= 3.0:
            return 0.
        incidence = float(-d@n/distance)
        if incidence <= .24:
            return 0.
        # The 8 cm prior and 18 pixel read threshold come from Flow's edge policy.
        pixels = focal*.08*incidence/distance
        margin = fov/2-.035
        if pixels < 5 or margin <= 0:
            return 0.
        tangent = np.array([-n[1],n[0]])
        visible = 0.
        for offset in (0.,-.76*width,.76*width):
            q = p + offset*tangent
            error = abs(wrap(angle(q-camera)-heading))
            if error < margin and grid.line_clear(camera,q+n*.09):
                visible += 1.-.22*(error/margin)**2
        return float((visible/3)*min(1.,pixels/18)*math.sqrt(incidence))

    def observe(self, grid, camera, endpoints, transform, focal, fov, extrinsics):
        self.camera = (focal,fov,*extrinsics)
        self.exposures += 1
        # Use current TF for each exposure; the map can shift after SLAM optimization.
        from osp_mission.r32_drive import in_map
        for key,_,_,width,kind,box in self.local:
            old = self.faces[key]
            p = np.asarray(in_map(('odom',*old.point),transform))
            if math.dist(p,camera[:2]) > 3.:
                continue
            n = axis(old.normal-transform[2])
            delta = endpoints-p
            supported = (abs(delta@n)<.10) & (abs(delta@np.array([-n[1],n[0]]))<width+.06)
            if np.count_nonzero(supported)<3:
                continue
            q = self.quality(grid,np.asarray(camera[:2]),camera[2],focal,fov,(key,p,n,width,kind,box))
            old.best = max(old.best,q)

    def decide(self, grid,cost,pose,transform,viewed,elapsed,duration,veto,choices=(),fitted=(),probe_only=False):
        if transform is None or self.camera is None or viewed is None:
            return super().decide(grid,cost,pose,transform,viewed,elapsed,duration,veto,choices,fitted,probe_only)
        faces = self.geometry(grid,transform,pose,fitted)
        clock_valid = lambda f: f.visits<2 and elapsed-f.last>=45.
        def guarded(goal):
            if veto(goal):
                return True
            if 'surface_odom' in goal:
                idx = self.register(np.asarray(goal['surface_odom']),goal['normal_odom'])
                return not clock_valid(self.faces[idx]) or self.faces[idx].best>=.88
            return False
        # Keep the existing priority between center, exploration frontiers and perimeter.
        base = super().decide(grid,cost,pose,transform,viewed,elapsed,duration,guarded,choices,fitted,probe_only)
        if probe_only or not faces:
            return base
        kind = 'wall' if base and base.get('r32_kind')=='wall_surface' else 'interior'
        eligible = [f for f in faces if f[4]==kind and self.faces[f[0]].best<.88 and clock_valid(self.faces[f[0]])]
        if not eligible:
            return base
        focal,fov,ex,ey,ea = self.camera
        robot = np.asarray(pose[:2])
        clear = clearance_grid(grid)
        start = grid.index(*robot)
        if not grid.inside(*start) or clear[start]<.32:
            return base
        safe,walk = grid.reachable(robot,.32)
        nav = r32.connected_nav_cells(cost,pose)
        best = None
        by_dist = sorted(eligible,key=lambda f: math.dist(robot,f[1]))
        for primary in by_dist[:14]:
            key,p,n,width,_,box = primary
            tangent = np.array([-n[1],n[0]])
            # Include oblique corner poses, retaining baseline close/narrow-strip sampling as fallback.
            stands,shifts = np.meshgrid([.45,.65,.9,1.15,1.40],[-.68,-.32,0.,.32,.68])
            pts = p+stands.ravel()[:,None]*n+shifts.ravel()[:,None]*tangent
            rows,cols = indices(grid,pts)
            rr,cc = indices(cost,pts)
            valid = ((rows>=0)&(cols>=0)&(rows<safe.shape[0])&(cols<safe.shape[1])&
                     (rr>=0)&(cc>=0)&(rr<nav.shape[0])&(cc<nav.shape[1]))
            options = []
            for i in np.flatnonzero(valid):
                cell = rows[i],cols[i]
                if not safe[cell] or not np.isfinite(walk[cell]) or not nav[rr[i],cc[i]]:
                    continue
                dist = walk[cell]
                options.append((dist+.4*abs(stands.ravel()[i]-.85),i,dist))
            # Keep a central pose and poses on either flank; avoid only evaluating near-identical points.
            buckets = {}
            for _,i,dist in sorted(options):
                buckets.setdefault(int(np.sign(shifts.ravel()[i])),(pts[i],dist))
            neighbors = [f for f in faces if math.dist(f[1],p)<3.8 and self.faces[f[0]].best<.88]
            for position,travel in buckets.values():
                aim = angle(p-position)
                for offset in (-.26,0.,.26):
                    heading = aim+offset-ea
                    camera = position + ex*axis(heading) + ey*axis(heading+math.pi/2)
                    gains = {}
                    for f in neighbors:
                        delta = max(0.,self.quality(grid,camera,heading+ea,focal,fov,f)-self.faces[f[0]].best)
                        if delta>=.13:
                            gains[f[0]] = delta
                    if key not in gains:
                        continue
                    # Preserve single-face baseline except for measured low-quality relooks.
                    if len(gains)<2 and not .08<self.faces[key].best<.70:
                        continue
                    score = sum(gains.values())/(1.+.32*travel+.16*abs(wrap(heading-pose[2])))
                    if score<.32 or (best and score<=best[0]):
                        continue
                    goal = dict(x=float(position[0]),y=float(position[1]),yaw=float(heading),
                                distance=math.dist(robot,position),features=np.zeros(5),stationary=False,
                                scan_angle=0.,r32_kind=kind+'_surface',surface_map=p.tolist(),
                                surface_odom=self.faces[key].point.tolist(),normal_odom=self.faces[key].normal,
                                nbv_face=key,nbv_gain=gains,nbv_score=float(score),clearance_m=float(clear[grid.index(*position)]))
                    if not guarded(goal):
                        best = score,goal
        if best:
            self.phase,self.reason = kind,'marker_multiview_gain'
            return best[1]
        return base

    def dispatched(self,goal,elapsed):
        super().dispatched(goal,elapsed)
        if 'surface_odom' in goal:
            old = self.faces[self.register(np.asarray(goal['surface_odom']),goal['normal_odom'])]
            old.visits += 1
            old.last = elapsed
