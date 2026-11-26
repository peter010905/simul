"""A bounded coarse perimeter pass on measured walls, then interior-only discovery."""
import math
import cv2
import numpy as np
from scipy import ndimage

from osp_mission.auto_chair_strip import chair_returns
from osp_mission.box_views import odom_point


def measured_perimeter(grid, pose):
    free = (grid.cells >= 0) & (grid.cells <= 25)
    chairs = chair_returns(grid)
    span = max(1, math.ceil(.40/grid.resolution))
    ribbon = ndimage.binary_closing(chairs, structure=np.ones((span,span),bool)) | chairs
    radius=max(1,math.ceil(.189/grid.resolution))
    yy,xx=np.ogrid[-radius:radius+1,-radius:radius+1]
    closure=xx*xx+yy*yy <= (.189/grid.resolution+1e-8)**2
    barrier=ribbon | ndimage.binary_dilation(chairs,structure=closure)
    labels, _ = ndimage.label(free & ~barrier)
    start = grid.index(*pose[:2])
    if not grid.inside(*start) or not labels[start]:
        return None
    # Occupied islands are holes; filling them excludes boxes from the outer lap.
    floor = ndimage.binary_fill_holes(labels == labels[start])
    contours, _ = cv2.findContours(floor.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    contour = max(contours,key=lambda c:cv2.arcLength(c,True))
    points = contour.reshape(-1,2)
    if len(points) < 12:
        return None
    support = (grid.cells >= 65) | ribbon
    distance = ndimage.distance_transform_edt(~support)*grid.resolution
    supported = distance[points[:,1],points[:,0]] <= .20
    return points, floor, supported, int(chairs.sum())


class WallLap:
    def __init__(self, budget_fraction=.30, maximum_seconds=180., waypoint_spacing=2.3):
        self.budget_fraction=budget_fraction
        self.maximum_seconds=maximum_seconds
        self.spacing=waypoint_spacing
        self.phase='perimeter'
        self.reason='initial_pass'
        self.active=False
        self.last_was_lap=False
        self.goal=None
        self.entry=None
        self.previous=None
        self.path=0.
        self.perimeter_m=0.
        self.normals=[]
        self.turn=0.
        self.missing_since=None
        self.last_support=0.
        self.chair_cells=0
        self.visited=[]

    def finish(self, reason):
        self.phase='interior'
        self.reason=reason
        self.active=False

    def position(self, x, y, elapsed):
        point=(x,y)
        if self.phase != 'perimeter':
            return
        if self.active and self.previous is not None:
            step=math.dist(point,self.previous)
            if step <= .5:
                self.path += step
        self.previous=point
        if not self.active or self.goal is None:
            return
        if math.dist(point,self.goal['lap_pose_odom']) <= .55 and not self.goal.get('credited'):
            self.goal['credited']=True
            if self.entry is None:
                self.entry=point
                self.path=0.
            angle=self.goal['lap_normal_odom']
            if self.normals:
                self.turn += math.remainder(angle-self.normals[-1],2*math.pi)
            self.normals.append(angle)
            self.visited.append(tuple(self.goal['lap_pose_odom']))
        if (self.entry is not None and self.path>=max(6.,.60*self.perimeter_m) and abs(self.turn)>=1.5*math.pi
                and len({round(a/(math.pi/2)) % 4 for a in self.normals}) >= 4
                and self.last_support>=.65 and math.dist(point,self.entry)<=1.2):
            self.finish('closed_perimeter_pass')

    def interrupted(self):
        self.active=False

    def dispatched(self, goal):
        self.last_was_lap=bool(goal.get('wall_lap'))
        self.active=self.phase=='perimeter' and self.last_was_lap
        self.goal=dict(goal) if self.active else None

    def expire(self, elapsed, duration):
        budget=min(self.maximum_seconds,self.budget_fraction*duration,max(0.,duration-240.))
        if self.phase == 'perimeter' and elapsed>=budget:
            self.finish('perimeter_time_budget')
        return budget

    def choose(self, grid, distances, pose, transform, elapsed, duration, veto):
        if self.phase != 'perimeter':
            return None
        budget=self.expire(elapsed,duration)
        if self.phase != 'perimeter':
            return None
        boundary=measured_perimeter(grid,pose)
        if boundary is None:
            if self.missing_since is None:
                self.missing_since=elapsed
            if elapsed-self.missing_since>=12.:
                self.finish('no_connected_measured_boundary')
            return None
        points,floor,supported,self.chair_cells=boundary
        self.last_support=float(supported.mean())
        world=np.array([grid.world(r,c) for c,r in points])
        delta=np.roll(world,-1,axis=0)-world
        lengths=np.linalg.norm(delta,axis=1)
        arc=np.r_[0.,np.cumsum(lengths[:-1])]
        perimeter=float(lengths.sum())
        self.perimeter_m=perimeter
        near=int(np.argmin(np.linalg.norm(world-pose[:2],axis=1)))
        step=max(1,round(.15/grid.resolution))
        best=None
        for i in range(0,len(points),step):
            advance=float((arc[i]-arc[near]) % perimeter)
            if not supported[i] or not .8 <= advance <= 4.0:
                continue
            tangent=world[(i+4)%len(world)]-world[(i-4)%len(world)]
            norm=np.linalg.norm(tangent)
            if norm < .05:
                continue
            tangent/=norm
            normal=np.array([-tangent[1],tangent[0]])
            plus=grid.index(*(world[i]+.4*normal))
            if not grid.inside(*plus) or not floor[plus]:
                normal=-normal
            for stand in (.9,1.2,1.5):
                point=world[i]+stand*normal
                cell=grid.index(*point)
                if (not grid.inside(*cell) or not floor[cell]
                        or not np.isfinite(distances[cell])):
                    continue
                walk=float(distances[cell])
                if not .65 <= walk <= min(5.5,.20*(budget-elapsed+8.)):
                    continue
                yaw=math.atan2(tangent[1],tangent[0])
                goal=dict(x=float(point[0]),y=float(point[1]),yaw=yaw,scan_angle=0.,
                          features=np.array([0.,0.,walk,0.,0.]),distance=walk,
                          stationary=False,face_inspection=False,r32_kind='wall_lap',wall_lap=True,
                          lap_pose_odom=odom_point(point,transform).tolist(),
                          lap_normal_odom=math.atan2(normal[1],normal[0])+transform[2],
                          lap_advance_m=advance,lap_wall_distance_m=stand)
                if veto(goal):
                    continue
                repeat=any(math.dist(goal['lap_pose_odom'],p)<.8 for p in self.visited[-8:])
                score=abs(advance-self.spacing)+.35*walk+.4*abs(stand-1.2)+2.*repeat
                if best is None or score<best[0]:
                    best=score,goal
        if best is not None:
            self.missing_since=None
            return best[1]
        if self.missing_since is None:
            self.missing_since=elapsed
        elif elapsed-self.missing_since>=12.:
            self.finish('no_safe_perimeter_goal')
        return None

    def report(self):
        return dict(phase=self.phase,reason=self.reason,path_m=round(self.path,2),
                    confirmed_stops=len(self.normals),turn_rad=round(self.turn,2),
                    boundary_support=round(self.last_support,3),chair_cells=self.chair_cells)
