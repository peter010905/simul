"""Short, evidence-limited look at a different side of a freestanding obstacle."""
import math

import numpy as np
from scipy import ndimage

from osp_mission.box_views import odom_point


def separation(a,b):
    return abs(math.remainder(a-b,math.tau))


class EnclosedSurface:
    """An island encircled by connected known floor; its interior can be unknown."""
    def __init__(self, grid, pose, anchor, window_m):
        self.grid=grid
        r,c=grid.index(*anchor)
        radius=math.ceil(window_m/grid.resolution)
        self.row,self.col=max(0,r-radius),max(0,c-radius)
        patch=grid.cells[self.row:r+radius+1,self.col:c+radius+1]
        self.labels=None
        pr,pc=grid.index(*pose[:2]);pr-=self.row;pc-=self.col
        if not (0<=pr<patch.shape[0] and 0<=pc<patch.shape[1]):return
        free=(patch>=0)&(patch<=25)
        if not free[pr,pc]:return
        connected,_=ndimage.label(free)
        floor=connected==connected[pr,pc]
        enclosed=ndimage.binary_fill_holes(floor)&~floor
        self.labels,_=ndimage.label(enclosed,np.ones((3,3),dtype=bool))
        self.solid=patch>=65

    def owner(self,position,normal):
        if self.labels is None:return None
        # Sample behind the boundary. Distinguish unknown inside an obstacle
        # from occupied sensor returns on its surface.
        inward=np.asarray(position)-.55*self.grid.resolution*normal
        gr,gc=self.grid.index(*inward);r,c=gr-self.row,gc-self.col
        rows,cols=self.labels.shape
        if not (0<=r<rows and 0<=c<cols):return None
        rr,cc=np.mgrid[max(0,r-1):min(rows,r+2),max(0,c-1):min(cols,c+2)]
        owners=np.unique(self.labels[rr,cc][self.solid[rr,cc]])
        if 0 in owners:return None  # return near an exterior wall or another unbounded body
        return int(owners[0]) if len(owners)==1 else None

    def center(self,label):
        r,c=np.where(self.labels==label)
        if not len(r):return None
        # A single pole or a transient speckle does not merit an extra turn.
        if len(r)*self.grid.resolution**2 < .035:return None
        return self.grid.world(self.row+float(np.mean(r)),self.col+float(np.mean(c)))


def continuation(state,grid,pose,transform,anchor,eligible,elapsed,budget,done,active_center):
    if budget<=0:return None
    angle=state.seed['normal']-transform[2]
    normal=np.array([math.cos(angle),math.sin(angle)])
    island=EnclosedSurface(grid,pose,anchor,3.8)
    owner=island.owner(anchor,normal)
    if owner is None:return None
    center=island.center(owner)
    if center is None:return None
    center=odom_point(center,transform)
    if active_center is not None and math.dist(center,active_center)>.30:return None
    if state.seed['center'] is not None and math.dist(center,state.seed['center'])>.30:return None
    if not done and any(0<=elapsed-t<90. and math.dist(center,c)<.65 for t,c in state.recent):return None
    checked=[]
    prior=(state.seed['normal'],*done)
    for opt in eligible:
        g=opt.goal
        if opt.seconds>budget or any(g.get(k) for k in ('previous_attempts','completed_box','tagged_box','reopened_face')):
            continue
        yaw=g.get('normal_odom');face=g.get('surface_map')
        if yaw is None or face is None or not math.isfinite(yaw):continue
        difference=min(separation(yaw,a) for a in prior)
        if difference<math.radians(50):continue
        normal=np.array([math.cos(yaw-transform[2]),math.sin(yaw-transform[2])])
        face=np.asarray(face);point=np.array([g['x'],g['y']])
        if island.owner(face,normal)!=owner:continue
        direction=point-face;distance=np.linalg.norm(direction)
        if distance<.25 or direction@normal < .60*distance or not grid.line_clear(point,face+.10*normal):continue
        distinct=min(difference,math.pi/2)/(math.pi/2)
        checked.append(((.5+distinct)/(2.+opt.seconds),opt,yaw))
    if not checked:return None
    _,option,yaw=max(checked,key=lambda record:record[0])
    return option,dict(survey_follow_host=list(center),survey_follow_normal=yaw,
                       survey_follow_nav=list(odom_point((option.goal['x'],option.goal['y']),transform)),
                       survey_follow_step=len(done)+1,survey_follow_budget_s=budget,
                       survey_follow_geometry='enclosed_surface')
