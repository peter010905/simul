"""A bounded coarse perimeter pass on measured walls, then interior-only discovery.

.50: lap goal headings follow the contour travel direction (they pointed
backwards before); after wall entry every continuation (contour or metric)
must advance in space along the previous lap direction, and a credited lap stop is never
selected again. A lap that cannot continue hands over to interior discovery
instead of idling until the perimeter time budget expires.
"""
import math
import cv2
import numpy as np
from scipy import ndimage

from osp_mission.auto_chair_strip import chair_returns
from osp_mission.box_views import odom_point

# .66 (from .62.1 far wall): lap stops 1.2-1.8 m from the measured wall, 1.5 m preferred.
LAP_STANDOFFS_M = (1.2, 1.5, 1.8)
LAP_PREFERRED_M = 1.5

# .65: a lap that is well along and still advancing at its time budget may continue up to here.
EXTENDED_LAP_S = 210.
EXTEND_MIN_PROGRESS = .60      # share of a full turn, or of the measured perimeter
EXTEND_RECENT_STOP_S = 25.     # a lap stop was credited this recently

# .72: one lap direction around the room centre. In four physical .67 runs (2026-11-28) the lap
# goals changed direction mid-lap (e.g. +23,+20 deg then -5,-28,-31 deg around the room centre)
# and covered only 56-173 deg in 150-218 s. The contour "forward" flips on the zig-zag floor contour
# of a desk/chair-leg band. After entry each lap stop must advance 3-75 deg around the centre of the
# measured floor in the direction fixed at the first continuation (.72.1: the contour orientation).
LAP_MIN_ADVANCE_RAD = math.radians(3.)
LAP_MAX_ADVANCE_RAD = math.radians(75.)


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
    def __init__(self, budget_fraction=.30, maximum_seconds=180., waypoint_spacing=2.3,
                 entry_wait_s=30., continuation_wait_s=15., revisit_radius_m=.8):
        self.budget_fraction=budget_fraction
        self.maximum_seconds=maximum_seconds
        self.spacing=waypoint_spacing
        # .51: callable(point)->bool for lap stops in front of a committed chair wall.
        # Those faces carry no markers, so the lap crosses them with long steps.
        self.sparse=None
        self.sparse_spacing=4.5
        self.sparse_stops=0
        # .52: callable(point)->point|None placing chair-face stops in front of the wall.
        self.project=None
        self.look_toward_wall=math.radians(45.)  # 45+90 at an inside corner stays within 135 deg
        self.entry_wait_s=entry_wait_s
        self.continuation_wait_s=continuation_wait_s
        self.revisit_radius_m=revisit_radius_m
        self.backward_rejected=0
        self.revisit_rejected=0
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
        self.anchor_was_unknown=False
        self.candidate_count=0
        self.tangent_odom=None
        self.selection_mode='none'
        self.visited=[]
        # .64: a lap stop interrupted before arrival (e.g. by a marker follow-up) is resumed once.
        self.pending=None
        self.clock=0.
        self.resumed_stops=0
        self.last_credit_s=None
        self.extended=False
        self.best_progress=0.
        self.sense=None            # .72: +1 counter-clockwise, -1 clockwise around the floor centre
        self.centre_odom=None
        self.angular=0.
        self.last_credit_angle=None
        self.angular_rejected=0

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
            self.last_credit_s=elapsed
            if self.entry is None:
                self.entry=point
                self.path=0.
            if self.centre_odom is not None:
                bearing=math.atan2(self.goal['lap_pose_odom'][1]-self.centre_odom[1],
                                   self.goal['lap_pose_odom'][0]-self.centre_odom[0])
                if self.last_credit_angle is not None and self.sense is not None:
                    self.angular+=max(0.,self.sense*math.remainder(bearing-self.last_credit_angle,2*math.pi))
                self.last_credit_angle=bearing
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
        previous=self.goal
        if goal.get('wall_lap'):
            self.pending=None
        elif (self.phase=='perimeter' and self.active and previous is not None
              and not previous.get('credited') and not previous.get('lap_resumed')):
            self.pending=dict(previous,interrupted_at=self.clock)
        self.last_was_lap=bool(goal.get('wall_lap'))
        self.active=self.phase=='perimeter' and self.last_was_lap
        self.goal=dict(goal) if self.active else None
        if self.active:
            self.tangent_odom=goal.get('lap_tangent_odom',self.tangent_odom)

    def progress(self):
        # .67: the furthest progress reached; a noisy stop normal must not undo it.
        # .72: the stop-normal turn jumped on zig-zag contours (1.01 after half a lap on the robot);
        # progress is the travelled share of the perimeter or the credited angle around the centre.
        path=self.path/self.perimeter_m if self.perimeter_m > 0 else 0.
        self.best_progress=max(self.best_progress,path,self.angular/(2*math.pi))
        return self.best_progress

    def expire(self, elapsed, duration):
        self.clock=elapsed
        budget=min(self.maximum_seconds,self.budget_fraction*duration,max(0.,duration-240.))
        if (self.phase == 'perimeter' and elapsed>=budget and self.entry is not None
                and self.progress()>=EXTEND_MIN_PROGRESS and self.last_credit_s is not None
                and elapsed-self.last_credit_s<=EXTEND_RECENT_STOP_S):
            budget=max(budget,min(EXTENDED_LAP_S,max(0.,duration-240.)))
            self.extended=self.extended or elapsed<budget
        if self.phase == 'perimeter' and elapsed>=budget:
            self.finish('perimeter_time_budget')
        return budget

    def _wait(self, elapsed, reason):
        if self.missing_since is None:
            self.missing_since=elapsed
        self.reason=reason
        limit=self.entry_wait_s if self.entry is None else self.continuation_wait_s
        if elapsed-self.missing_since>=limit:
            self.finish('no_safe_perimeter_goal' if self.entry is None
                        else 'perimeter_continuation_unavailable')
        return None

    def resume(self, pose, transform, elapsed, veto, max_age_s=90., max_distance_m=4.):
        """The interrupted, never reached lap stop, in the current map frame (once)."""
        stop, self.pending = self.pending, None
        if stop is None or elapsed-stop['interrupted_at'] > max_age_s:
            return None
        x, y, a = transform
        dx, dy = stop['lap_pose_odom'][0]-x, stop['lap_pose_odom'][1]-y
        c, s = math.cos(a), math.sin(a)
        goal = {k: v for k, v in stop.items() if k not in ('interrupted_at', 'credited')}
        goal.update(x=c*dx+s*dy, y=-s*dx+c*dy, lap_resumed=True, wall_entry=False)
        if math.dist((goal['x'], goal['y']), pose[:2]) > max_distance_m or veto(goal):
            return None
        self.resumed_stops += 1
        self.missing_since = None
        self.reason = 'resume_interrupted_lap_stop'
        return goal

    def choose(self, grid, distances, pose, transform, elapsed, duration, veto):
        if self.phase != 'perimeter':
            return None
        budget=self.expire(elapsed,duration)
        if self.phase != 'perimeter':
            return None
        resumed=self.resume(pose,transform,elapsed,veto)
        if resumed is not None:
            return resumed
        boundary=measured_perimeter(grid,pose)
        if boundary is None:
            return self._wait(elapsed,'waiting_for_measured_wall_entry')
        points,floor,supported,self.chair_cells=boundary
        self.last_support=float(supported.mean())
        world=np.array([grid.world(r,c) for c,r in points])
        delta=np.roll(world,-1,axis=0)-world
        lengths=np.linalg.norm(delta,axis=1)
        arc=np.r_[0.,np.cumsum(lengths[:-1])]
        perimeter=float(lengths.sum())
        self.perimeter_m=perimeter
        nearest=int(np.argmin(np.linalg.norm(world-pose[:2],axis=1)))
        self.anchor_was_unknown=not bool(supported[nearest])
        measured=np.flatnonzero(supported)
        if not len(measured):
            return self._wait(elapsed,'waiting_for_measured_wall_entry')
        # An unfinished map has nearby unknown edges. They cannot anchor
        # progress along an actually observed wall several contour-metres away.
        near=int(measured[np.argmin(np.linalg.norm(world[measured]-pose[:2],axis=1))])
        self.candidate_count=0
        step=max(1,round(.15/grid.resolution))
        free=(grid.cells>=0)&(grid.cells<=25)
        clearance=ndimage.distance_transform_edt(np.pad(free,1))[1:-1,1:-1]*grid.resolution
        lines=cv2.HoughLinesP(np.uint8(grid.cells>=65)*255,1,math.pi/180,
                             threshold=max(8,round(.6/grid.resolution)),
                             minLineLength=1.2/grid.resolution,maxLineGap=.4/grid.resolution)
        guides=[]
        for x1,y1,x2,y2 in (() if lines is None else lines[:,0]):
            a,b=grid.world(y1,x1),grid.world(y2,x2)
            length=float(np.linalg.norm(b-a))
            if length>=1.2:
                guides.append((a,(b-a)/length,length))
        reference_angle=(self.tangent_odom-transform[2] if self.tangent_odom is not None else pose[2] if len(pose)>2 else 0.)
        reference=np.array([math.cos(reference_angle),math.sin(reference_angle)])
        best=None
        self.selection_mode='none'
        self.backward_rejected=self.revisit_rejected=self.angular_rejected=0
        # A reference exists after entry: the last dispatched lap tangent.
        forward_required=self.entry is not None
        rows,cols=np.nonzero(floor)
        centre=grid.world(float(rows.mean()),float(cols.mean())) if len(rows) else None
        robot_bearing=None
        if centre is not None:
            self.centre_odom=odom_point(centre,transform).tolist()
            offset=np.asarray(pose[:2])-centre
            robot_bearing=math.atan2(offset[1],offset[0])
            if forward_required and self.sense is None:
                # .72.1: the direction of increasing contour arc (the contour's own orientation),
                # which contour continuations always follow. .72 took it from the entry tangent,
                # which could point the other way and then rejected every continuation (Gazebo
                # 3801/3804: lap ended at 81/97 s with one to five stops).
                closed=np.roll(world,-1,axis=0)
                area=float((world[:,0]*closed[:,1]-closed[:,0]*world[:,1]).sum())
                self.sense=1 if area>=0. else -1
        for metric in (False,True):
            for i in range(0,len(points),step):
                advance=float((arc[i]-arc[near]) % perimeter)
                if not supported[i]:
                    continue
                entering=self.entry is None
                if not entering and not metric and not .8 <= advance <= 4.0:
                    continue
                if entering and np.linalg.norm(world[i]-pose[:2]) > 4.:
                    continue
                tangent=world[(i+4)%len(world)]-world[(i-4)%len(world)]
                norm=np.linalg.norm(tangent)
                if norm < .05:
                    continue
                tangent/=norm
                # Direction of increasing contour arc, i.e. of positive advance.
                travel=tangent.copy()
                nearby=[]
                for anchor,direction,length in guides:
                    projection=float((world[i]-anchor)@direction)
                    residual=float(np.linalg.norm(world[i]-(anchor+np.clip(projection,0,length)*direction)))
                    if -.25<=projection<=length+.25 and residual<=.45:
                        nearby.append((residual+.05/length,direction))
                if nearby:
                    tangent=min(nearby,key=lambda pair:pair[0])[1]
                normal=np.array([-tangent[1],tangent[0]])
                plus=grid.index(*(world[i]+.4*normal))
                if not grid.inside(*plus) or not floor[plus]:
                    normal=-normal
                tangent=np.array([normal[1],-normal[0]])
                # .50: the fixed-handedness tangent pointed against the travel
                # direction on real contours (goal yaw 180 deg from motion), so
                # every stop ended facing backwards and the metric fallback used
                # a reversed reference. Orient it along the contour travel.
                if float(tangent@travel)<0.:
                    tangent=-tangent
                if (metric or forward_required) and float(tangent@reference)<-.70:
                    self.backward_rejected+=1
                    continue
                for stand in LAP_STANDOFFS_M:
                    point=world[i]+stand*normal
                    if self.project is not None:
                        shifted=self.project(point)
                        if shifted is not None:
                            point=np.asarray(shifted,float)
                    cell=grid.index(*point)
                    if (not grid.inside(*cell) or not floor[cell]
                            or clearance[cell]<.32 or not np.isfinite(distances[cell])):
                        continue
                    walk=float(distances[cell])
                    if not .65 <= walk <= min(5.5,.20*(budget-elapsed+8.)):
                        continue
                    displacement=point-np.asarray(pose[:2])
                    if forward_required and self.sense is not None and robot_bearing is not None:
                        around=self.sense*math.remainder(
                            math.atan2(point[1]-centre[1],point[0]-centre[0])-robot_bearing,2*math.pi)
                        if not LAP_MIN_ADVANCE_RAD<=around<=LAP_MAX_ADVANCE_RAD:
                            self.angular_rejected+=1
                            continue
                    if metric and (float(displacement@reference)<-.30 or float(displacement@tangent)<.55):
                        self.backward_rejected+=1
                        continue
                    # Contour advance already moves along the arc; this bound only
                    # rejects real spatial reversals while allowing the standoff
                    # shift of an inside corner.
                    if forward_required and not metric and float(displacement@reference)<-.80:
                        self.backward_rejected+=1
                        continue
                    yaw=math.atan2(tangent[1],tangent[0])
                    # .55: arrive turned toward the wall by look_toward_wall so the camera
                    # covers the wall section ahead. The progress reference stays the travel
                    # tangent. With a heading along the wall a wall print never entered the
                    # image (0 of ~250 nearby samples in two Gazebo layouts).
                    look=(math.cos(self.look_toward_wall)*tangent-math.sin(self.look_toward_wall)*normal)
                    look_yaw=math.atan2(look[1],look[0])
                    goal=dict(x=float(point[0]),y=float(point[1]),yaw=look_yaw,scan_angle=0.,
                              features=np.array([0.,0.,walk,0.,0.]),distance=walk,
                              stationary=False,face_inspection=False,r32_kind='wall_lap',wall_lap=True,
                              lap_pose_odom=odom_point(point,transform).tolist(),
                              lap_normal_odom=math.atan2(normal[1],normal[0])+transform[2],
                              lap_tangent_odom=yaw+transform[2],
                              lap_advance_m=advance,lap_wall_distance_m=stand)
                    near_visit=any(math.dist(goal['lap_pose_odom'],p)<self.revisit_radius_m for p in self.visited)
                    if near_visit and not entering:
                        self.revisit_rejected+=1
                        continue
                    if veto(goal):
                        continue
                    repeat=near_visit
                    self.candidate_count+=1
                    if entering:
                        # Reach a measured wall safely first. An endpoint also
                        # advances along it when it offers a short connected route.
                        forward=float((point-np.asarray(pose[:2]))@tangent)
                        score=walk+.5*abs(forward-1.2)+.4*abs(stand-LAP_PREFERRED_M)+2.*repeat
                    chair_face=bool(self.sparse is not None and self.sparse(point))
                    spacing=self.sparse_spacing if chair_face else self.spacing
                    if entering:
                        pass
                    elif metric:
                        score=abs(walk-spacing)+.3*walk+.5*(1.-float(tangent@reference))+2.*repeat
                    else:
                        score=abs(advance-spacing)+.35*walk+.4*abs(stand-LAP_PREFERRED_M)+2.*repeat
                    goal['lap_chair_face']=chair_face
                    if best is None or score<best[0]:
                        best=score,goal
                        self.selection_mode='metric_wall' if metric else 'contour'
            if best is not None:
                break
        if best is not None:
            self.missing_since=None
            self.reason='entering_measured_wall' if self.entry is None else 'following_measured_wall'
            best[1]['wall_entry']=self.entry is None
            return best[1]
        return self._wait(elapsed,'waiting_for_safe_wall_entry' if self.entry is None
                          else 'waiting_for_safe_wall_continuation')

    def report(self):
        return dict(phase=self.phase,reason=self.reason,path_m=round(self.path,2),
                    confirmed_stops=len(self.normals),turn_rad=round(self.turn,2),
                    boundary_support=round(self.last_support,3),chair_cells=self.chair_cells,
                    entered_wall=self.entry is not None,nearest_edge_unmeasured=self.anchor_was_unknown,
                    safe_wall_candidates=self.candidate_count,selection_mode=self.selection_mode,
                    backward_rejected=self.backward_rejected,revisit_rejected=self.revisit_rejected,
                    resumed_stops=self.resumed_stops,extended=self.extended,
                    lap_sense=self.sense,angular_progress=round(self.angular/(2*math.pi),2),
                    angular_rejected=self.angular_rejected,
                    progress=round(self.progress(),2),
                    waiting_since_s=None if self.missing_since is None else round(self.missing_since,1))
