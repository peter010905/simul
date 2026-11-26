"""Briefly inspect additional surfaces after actual arrival at a compact object."""
from collections import deque
import math
import numpy as np
from scipy import ndimage

from osp_mission.box_views import local_boxes, odom_point


def angle_gap(a,b):
    return abs(math.remainder(a-b,2*math.pi))


def face_match(faces, point, yaw, resolution):
    """Return an unambiguous fitted plane, tolerating rasterized surface normals."""
    matches=[]
    for index,(face,normal) in enumerate(faces):
        opposite=angle_gap(yaw, math.atan2(normal[1],normal[0]))
        if opposite>math.radians(58):
            continue
        tangent=np.array([-normal[1],normal[0]])
        half_width=max(abs((other-face)@tangent) for other,_ in faces)
        delta=point-face
        plane=abs(delta@normal)
        if plane<=max(.10,2.2*resolution) and abs(delta@tangent)<=half_width+resolution:
            matches.append((plane+.08*opposite,index))
    matches.sort()
    if not matches or (len(matches)>1 and matches[1][0]-matches[0][0]<resolution*.5):
        return None
    return matches[0][1]


def isolated_fit(grid, pose, anchor, known_center):
    choices=[]
    for center,faces in local_boxes(grid,pose):
        distance=float(np.linalg.norm(anchor-center))
        radius=max(float(np.linalg.norm(face-center)) for face,_ in faces)
        if distance<=radius+max(.15,3*grid.resolution):
            choices.append((distance,center,faces))
    if len(choices)!=1:
        return None
    _,center,faces=choices[0]
    if known_center is not None:
        return (center,faces) if math.dist(center,known_center)<.30 else None
    # Require an independently compact component for a formerly generic goal.
    # Limit the test to a local patch. A component leaving the patch cannot qualify.
    row,col=grid.index(*center)
    radius=int(math.ceil(1.25/grid.resolution))
    r0,c0=max(0,row-radius),max(0,col-radius)
    patch=grid.cells[r0:row+radius+1,c0:col+radius+1]>=65
    labels,_=ndimage.label(patch,np.ones((3,3),np.uint8))
    rows,cols=np.nonzero(labels)
    close=np.hypot(rows+r0-row,cols+c0-col)*grid.resolution<=.65
    owners=np.unique(labels[rows[close],cols[close]])
    if len(owners)!=1:
        return None
    r,c=np.where(labels==owners[0])
    spans=(np.ptp(r)+1)*grid.resolution,(np.ptp(c)+1)*grid.resolution
    if (min(spans)<.2 or max(spans)>1.65 or min(r.min(),c.min())==0 or
            r.max()==patch.shape[0]-1 or c.max()==patch.shape[1]-1):
        return None
    return center,faces


class FollowThrough:
    def __init__(self):
        self.seed=None
        self.window=None
        self.recent=deque(maxlen=24)

    def dispatched(self,goal,elapsed):
        if 'survey_follow_host' in goal:
            center=goal['survey_follow_host']
            if self.window is None:
                self.window=(elapsed,[],tuple(center))
                self.recent.append((elapsed,center))
            self.window[1].append(goal['survey_follow_normal'])
            self.seed['nav']=tuple(goal['survey_follow_nav'])
            return
        self.window=None
        self.seed=None
        if (goal.get('r32_kind')=='interior_surface' and 'normal_odom' in goal
                and 'survey_nav_odom' in goal and not goal.get('completed_box')
                and not goal.get('tagged_box')):
            self.seed=dict(anchor=tuple(goal['surface_odom']), normal=goal['normal_odom'],
                           nav=tuple(goal['survey_nav_odom']), center=goal.get('batch_box'),
                           expires=elapsed+min(95.,max(12.,goal.get('survey_expected_s',0.)+12.)))

    def choose(self,grid,pose,transform,eligible,waiting,period,elapsed,remaining):
        if self.seed is None or not eligible.get('interior'):
            return None
        seed=self.seed
        start,done,active_center=self.window if self.window is not None else (elapsed,[],None)
        if len(done)>=2 or (not done and elapsed>seed['expires']):
            return None
        robot=odom_point(pose[:2],transform)
        if math.dist(robot,seed['nav'])>.65 or math.dist(robot,seed['anchor'])>2.3:
            return None
        # Do not delay service beyond its existing class/deadline budget.
        budget=min(remaining-2.,30.-(elapsed-start),*(period-(elapsed-waiting[kind])
                   for kind in ('wall','frontier') if eligible.get(kind)))
        if budget<=0:
            return None
        tx,ty,a=transform
        def mapped(point):
            dx,dy=point[0]-tx,point[1]-ty
            return np.array([math.cos(a)*dx+math.sin(a)*dy,-math.sin(a)*dx+math.cos(a)*dy])
        anchor=mapped(seed['anchor'])
        box=isolated_fit(grid,pose,anchor,mapped(seed['center']) if seed['center'] is not None else None)
        if box is None:
            return None
        center,faces=box
        odom_center=odom_point(center,transform)
        if active_center is not None and math.dist(odom_center,active_center)>.30:
            return None
        if not done and any(elapsed-t<90. and math.dist(odom_center,c)<.65 for t,c in self.recent):
            return None
        prior=face_match(faces,anchor,seed['normal']-a,grid.resolution)
        if prior is None:
            return None
        options=[]
        for opt in eligible['interior']:
            g=opt.goal
            if (opt.seconds>budget or g.get('previous_attempts') or g.get('completed_box')
                    or g.get('tagged_box') or g.get('reopened_face')):
                continue
            index=face_match(faces,np.asarray(g['surface_map']),g['normal_odom']-a,grid.resolution)
            if index is None or index==prior:
                continue
            face,normal=faces[index]
            norm=math.atan2(normal[1],normal[0])+a
            if any(angle_gap(norm,seen)<math.radians(50) for seen in done):
                continue
            delta=np.asarray([g['x'],g['y']])-face
            if delta@normal < .60*np.linalg.norm(delta) or not grid.line_clear((g['x'],g['y']),face+.10*normal):
                continue
            options.append((opt.seconds,float(np.linalg.norm(np.asarray(g['surface_map'])-face)),opt,norm))
        if not options:
            return None
        _,_,opt,norm=min(options,key=lambda item:item[:2])
        return opt,dict(survey_follow_host=list(odom_center), survey_follow_normal=norm,
                        survey_follow_nav=list(odom_point((opt.goal['x'],opt.goal['y']),transform)),
                        survey_follow_step=len(done)+1,survey_follow_budget_s=budget)
