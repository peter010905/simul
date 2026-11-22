"""2D policy-search surrogate. Not Gazebo, physics or an ArUco image decoder.

Ground truth is private to the sensor/evaluation environment. The policy receives
only ray-observed occupancy, camera-inspected surfaces and past visited poses.
"""
import math
import numpy as np
from scipy.ndimage import distance_transform_edt
from osp_mission.scenarios import rasterize
from osp_mission.policy import options, choose, scan_angle, wrap


BODY_RADIUS = .22


def motion_blocked(observed, resolution, x, y, radius=.30, cell=None):
    """Conservative local shield, using only observed obstacle cells.

    The collision metric uses clearance at the containing cell centre. Check
    both that centre and the continuous destination so a cell-boundary step
    cannot pass the shield and immediately collide with an already seen wall.
    """
    iy, ix = (int(math.floor(y / resolution)), int(math.floor(x / resolution))) if cell is None else cell
    if not (0 <= iy < observed.shape[0] and 0 <= ix < observed.shape[1]):
        return True
    reach = int(math.ceil(radius / resolution)) + 1
    y0, y1 = max(0, iy - reach), min(observed.shape[0], iy + reach + 1)
    x0, x1 = max(0, ix - reach), min(observed.shape[1], ix + reach + 1)
    yy, xx = np.indices((y1 - y0, x1 - x0))
    continuous = (((xx + x0 + .5) * resolution - x) ** 2
                  + ((yy + y0 + .5) * resolution - y) ** 2) < radius ** 2
    discrete = ((xx + x0 - ix) ** 2 + (yy + y0 - iy) ** 2) * resolution ** 2 < BODY_RADIUS ** 2
    return bool(np.any((observed[y0:y1, x0:x1] > 50) & (continuous | discrete)))


class SearchEnv:
    def __init__(self,cfg,seed=0,resolution=.20,planner='legacy',camera_width=320,detection_dropout=.08,camera_hz=None,inspection_frames=3,frame_quarter_turns=0,view_standoff=None,lidar_samples=120,lidar_body_frame=False,raycast_backend='numpy',continuous_shortcuts=True,travel_glance_threshold=.75,angular_profile='legacy',max_turn_speed=1.,max_turn_acceleration=3.2,optimize_scan_alignment=False):
        if isinstance(lidar_samples,bool) or not isinstance(lidar_samples,int) or not 32<=lidar_samples<=2048:
            raise ValueError('LiDAR samples must be an integer between 32 and 2048')
        if not isinstance(lidar_body_frame,bool):raise ValueError('LiDAR frame mode must be a boolean')
        self.lidar_body_frame=lidar_body_frame
        if raycast_backend not in ('numpy','native'):raise ValueError('Unknown raycast backend')
        self.raycast_backend=raycast_backend
        if not isinstance(continuous_shortcuts,bool):raise ValueError('Shortcut mode must be a boolean')
        self.continuous_shortcuts=continuous_shortcuts
        if angular_profile not in ('legacy','trapezoid'):raise ValueError('Unknown angular motion profile')
        if not all(math.isfinite(v) and v>0 for v in (max_turn_speed,max_turn_acceleration)):
            raise ValueError('Angular motion limits must be finite and positive')
        self.angular_profile=angular_profile;self.max_turn_speed=max_turn_speed;self.max_turn_acceleration=max_turn_acceleration
        if not isinstance(optimize_scan_alignment,bool):raise ValueError('Scan alignment optimization must be Boolean')
        self.optimize_scan_alignment=optimize_scan_alignment
        self.observation_alignment_seconds=0.;self.observation_sweep_seconds=0.;self.avoidable_alignment_radians=0.
        if not 1<=inspection_frames<=20:raise ValueError('Invalid inspection frame count')
        self.inspection_frames=inspection_frames
        if camera_hz is not None and camera_hz<=0:raise ValueError('Invalid camera frequency')
        self.camera_period=None if camera_hz is None else 1./camera_hz
        self.last_camera=-float('inf')
        if not 0<=detection_dropout<1:raise ValueError('Invalid detection dropout')
        self.detection_dropout=detection_dropout;self.deadline=float('inf')
        self._lidar_pose=None;self._lidar_cache=None;self._normal_grid=None;self._normal_cache=None
        self.camera_focal=camera_width/(2*math.tan(1.25/2))
        self.planner=planner
        if not math.isfinite(travel_glance_threshold) or not .1<=travel_glance_threshold<=10.:
            raise ValueError('Travel glance threshold must be between 0.1 and 10 metres of visible faces')
        if planner=='edge_glance' and camera_hz is None:
            raise ValueError('Travel glances require an explicit camera clock')
        self.travel_glance_threshold=travel_glance_threshold
        self._last_glance_distance=0.;self._last_glance_time=-float('inf');self.travel_glances=0
        if view_standoff is not None and (planner not in ('surface_endpoint','surface_deadline') or not .5<=view_standoff<=3.):
            raise ValueError('View standoff is supported from 0.5 to 3 m by surface endpoint planners')
        self.directional_coverage=planner in ('directional','face_candidates','object_balanced','face_object','gaze','gaze_grid')
        self.edge_coverage=None
        if planner in ('edge','edge_glance','view_greedy','view_route','view_tour','plane_edge','plane_view','geometry_edge','geometry_view','symmetric_edge','geometry_hybrid'):
            from osp_mission.edge_policy import options as edge_options,choose as edge_choose,EdgeCoverage
            self.options,self.choose=edge_options,edge_choose
            self.edge_coverage=EdgeCoverage()
            if planner in ('view_greedy','view_route','view_tour','plane_view','geometry_view'):
                from osp_mission import view_route_policy
                self.options=view_route_policy.options
                self.choose=view_route_policy.choose if planner=='view_route' else view_route_policy.choose_greedy
                if planner=='view_tour':
                    from osp_mission.view_tour_policy import choose as tour_choose
                    self.choose=tour_choose
            if planner in ('plane_edge','plane_view','geometry_edge','geometry_view','symmetric_edge','geometry_hybrid'):
                from functools import partial
                from osp_mission.plane_views import sample,sample_all,symmetric_grid,hybrid
                sampler=hybrid if planner=='geometry_hybrid' else symmetric_grid if planner=='symmetric_edge' else sample_all if planner.startswith('geometry') else sample
                self.options=partial(self.options,point_sampler=sampler)
        elif planner=='surface_deadline':
            from osp_mission.surface_endpoint_policy import options as surface_options
            from osp_mission.time_budget import choose as budget_choose
            self.options=surface_options
            if view_standoff is not None:
                from functools import partial
                self.options=partial(surface_options,preferred_clearance=view_standoff)
            self.choose=lambda candidates,weights:budget_choose(candidates,weights,self.deadline-self.elapsed)
        elif planner in ('face_candidates','object_balanced','face_object','gaze','gaze_grid','endpoint_faces','endpoint_prior','endpoint_prior_faces'):
            from functools import partial
            from osp_mission.face_policy import options as face_options, choose as face_choose, choose_timed
            self.options=partial(face_options,face_candidates=planner not in ('object_balanced','gaze_grid','endpoint_prior'),balanced=planner in ('object_balanced','face_object'),adaptive_scans=planner in ('gaze','gaze_grid'),directional=planner not in ('endpoint_faces','endpoint_prior','endpoint_prior_faces'),probabilistic=planner in ('endpoint_prior','endpoint_prior_faces'))
            self.choose=choose_timed if planner in ('gaze','gaze_grid') else face_choose
        elif planner=='directional':
            from osp_mission.directional_policy import options as surface_options, choose as surface_choose
            self.options,self.choose=surface_options,surface_choose
        elif planner=='surface_endpoint':
            from osp_mission.surface_endpoint_policy import options as surface_options, choose as surface_choose
            self.options,self.choose=surface_options,surface_choose
            if view_standoff is not None:
                from functools import partial
                self.options=partial(surface_options,preferred_clearance=view_standoff)
        elif planner=='surface_v2':
            from osp_mission.surface_v2_policy import options as surface_options, choose as surface_choose
            self.options,self.choose=surface_options,surface_choose
        elif planner=='surface_balanced':
            from osp_mission.surface_balanced_policy import options as surface_options, choose as surface_choose
            self.options,self.choose=surface_options,surface_choose
        elif planner=='surface':
            from osp_mission.surface_policy import options as surface_options, choose as surface_choose
            self.options,self.choose=surface_options,surface_choose
        elif planner=='inspection':
            from osp_mission.inspection_policy import options as inspect_options, choose as inspect_choose
            self.options,self.choose=inspect_options,inspect_choose
        elif planner=='legacy': self.options,self.choose=options,choose
        else: raise ValueError('Unknown planner')
        self.cfg=cfg; self.res=resolution; self.truth=rasterize(cfg,resolution)
        if frame_quarter_turns:
            from osp_mission.frame_variation import rotate_scene_frame
            self.cfg,self.truth=rotate_scene_frame(cfg,self.truth,resolution,frame_quarter_turns)
        self.clearance=distance_transform_edt(~self.truth)*resolution
        self.map=np.full(self.truth.shape,-1,dtype=np.int8)
        self.viewed=np.zeros_like(self.truth); self.view_counts=np.zeros_like(self.truth,dtype=np.uint8); self.last_surface=-1.; self.x,self.y,self.yaw=self.cfg['start']
        if self.directional_coverage:
            self.viewed=np.zeros((8,)+self.truth.shape,dtype=bool);self.view_counts=np.zeros_like(self.viewed,dtype=np.uint8)
        if self.edge_coverage is not None:self.viewed=self.edge_coverage.grid(self.map,self.res,(0.,0.))
        self.trace=[]; self.elapsed=0.; self.found=set(); self.collisions=0; self.safety_stops=0; self.distance=0.; self.visits=[]; self.steps=0
        self.confirmations={}; self.last_confirmation={}
        self.rng=np.random.default_rng(seed)
        self.speed=float(self.rng.uniform(.22,.31)); self.turn_speed=float(self.rng.uniform(.45,.65))
        self.ranges=np.arange(.0,12.,resolution*.7)
        self.angles=np.linspace(-math.pi,math.pi,lidar_samples,endpoint=False)
        self._scan_angles=self.angles
        self.scan()

    def _normals(self,normal_function):
        occupied=self.map>50
        if self._normal_grid is None or not np.array_equal(occupied,self._normal_grid):
            self._normal_grid=occupied
            self._normal_cache=normal_function(self.map,self.res)
        return self._normal_cache

    def _lidar(self):
        # The old fast model fixed beam angles in world coordinates. In body
        # frame mode, an in-place turn rotates the finite set of LiDAR beams.
        pose=(self.x,self.y,self.yaw) if self.lidar_body_frame else (self.x,self.y)
        if pose==self._lidar_pose:return self._lidar_cache
        self._scan_angles=self.angles+self.yaw if self.lidar_body_frame else self.angles
        from osp_mission.raycast import cast
        self._lidar_cache=cast(self.truth,self.map,self.x,self.y,self.res,self._scan_angles,self.ranges,self.raycast_backend)
        self._lidar_pose=pose
        return self._lidar_cache

    def scan(self):
        self.trace.append((self.x,self.y,self.yaw,self.elapsed))
        end_y,end_x,first,inside,hits=self._lidar()
        if self.camera_period is not None:
            if self.elapsed-self.last_camera<self.camera_period-1e-9:return
            self.last_camera=self.elapsed
        angles=self._scan_angles
        camera=(np.abs((angles-self.yaw+math.pi)%(2*math.pi)-math.pi)<1.25/2)
        if self.edge_coverage is not None:
            valid=hits&inside
            distances=self.ranges[first[valid]];beams=angles[valid]
            endpoints=np.column_stack((self.x+distances*np.cos(beams),self.y+distances*np.sin(beams)))
            self.edge_coverage.observe(self.map,self.res,(0.,0.),(self.x,self.y,self.yaw),endpoints,
                self.elapsed,self.camera_focal,1.25)
            self.viewed=self.edge_coverage.viewed
        elif self.directional_coverage:
            from osp_mission.directional_policy import normals,direction_bins
            ny,nx=self._normals(normals)
            k=np.flatnonzero(camera);end=first[k];ys,xs=end_y[k],end_x[k]
            incidence=np.abs(np.cos(angles[k])*nx[ys,xs]+np.sin(angles[k])*ny[ys,xs])
            reliable=(incidence>.6)&(self.ranges[end]<.85*self.camera_focal*.12*incidence/18)&hits[k]&inside[k]
            if self.elapsed-self.last_surface>=.15:
                bins=direction_bins(-np.sin(angles[k]),-np.cos(angles[k]))
                cells=np.unique(np.stack((bins[reliable],ys[reliable],xs[reliable]),axis=1),axis=0)
                if len(cells):
                    b,y,x=cells.T;self.view_counts[b,y,x]=np.minimum(self.view_counts[b,y,x]+1,self.inspection_frames)
                    confirmed=self.view_counts[b,y,x]>=self.inspection_frames;b,y,x=b[confirmed],y[confirmed],x[confirmed]
                    for offset in [-1,0,1]:self.viewed[(b+offset)%8,y,x]=True
                self.last_surface=self.elapsed
        elif self.planner in ('surface','surface_balanced','surface_v2','surface_endpoint','endpoint_faces','endpoint_prior','endpoint_prior_faces','surface_deadline'):
            from osp_mission.surface_policy import normals
            ny,nx=self._normals(normals)
            k=np.flatnonzero(camera);end=first[k];ys,xs=end_y[k],end_x[k]
            incidence=-(np.cos(angles[k])*nx[ys,xs]+np.sin(angles[k])*ny[ys,xs])
            reliable=(incidence>.6)&(self.ranges[end]<.85*self.camera_focal*.12*incidence/18)&hits[k]&inside[k]
            if self.elapsed-self.last_surface>=.15:
                cells=np.unique(np.stack((ys[reliable],xs[reliable]),axis=1),axis=0)
                if len(cells):
                    y,x=cells.T;self.view_counts[y,x]=np.minimum(self.view_counts[y,x]+1,3)
                    self.viewed[y,x]=self.view_counts[y,x]>=3
                self.last_surface=self.elapsed
        else:
            for k in np.flatnonzero(camera):
                end=first[k]
                if self.ranges[end]<(2.5 if self.planner=='inspection' else 3.5) and hits[k] and inside[k]: self.viewed[end_y[k],end_x[k]]=True
        self.detect()

    def detect(self):
        if self.elapsed>self.deadline:return
        for target in self.cfg['targets']:
            ident=target['id']
            if ident in self.found: continue
            dx=target['x']-self.x; dy=target['y']-self.y; dist=math.hypot(dx,dy)
            angle=math.atan2(dy,dx)
            if dist>.95*self.camera_focal*target['size']/18 or dist<.25 or abs(wrap(angle-self.yaw))>1.25/2: continue
            # Detector depth/LiDAR localization accepts optical z below 8 m.
            if dist*math.cos(wrap(angle-self.yaw))>=8.:continue
            face=((self.x-target['x'])*math.cos(target['yaw'])+(self.y-target['y'])*math.sin(target['yaw']))/dist
            if face<.45 or dist>.95*self.camera_focal*target['size']*face/18: continue
            # Exclude the last 20 cm so the marker's own support is not an occluder.
            ds=np.arange(.1,max(.11,dist-.20),self.res*.35)
            xs=((self.x+ds*dx/dist)/self.res).astype(int).clip(0,self.map.shape[1]-1)
            ys=((self.y+ds*dy/dist)/self.res).astype(int).clip(0,self.map.shape[0]-1)
            if not self.truth[ys,xs].any() and self.rng.random()>self.detection_dropout and self.elapsed-self.last_confirmation.get(ident,-1.)>=.15:
                self.last_confirmation[ident]=self.elapsed
                self.confirmations[ident]=self.confirmations.get(ident,0)+1
                if self.confirmations[ident]>=3:self.found.add(ident)

    def rotate(self,angle,deadline=600.):
        if self.angular_profile=='trapezoid':
            from osp_mission.angular_motion import RotationProfile
            profile=RotationProfile(angle,self.max_turn_speed,self.max_turn_acceleration)
            elapsed=0.;position=0.
            while elapsed<profile.duration-1e-9 and self.elapsed<deadline and len(self.found)<3:
                delta=min(self.camera_period or .1,profile.duration-elapsed,deadline-self.elapsed)
                elapsed+=delta;fresh=profile.position(elapsed)
                self.yaw=wrap(self.yaw+math.copysign(fresh-position,angle));position=fresh
                self.elapsed+=delta;self.scan()
            return
        # Sub-frame angular steps combined with the camera gate can skip every
        # other frame (e.g. 1 rad/s at 5 Hz). Advance by one camera interval;
        # the final partial turn is still clipped to its angle and deadline.
        increment=math.copysign(self.turn_speed*self.camera_period if self.camera_period else math.pi/16,angle)
        remaining=abs(angle)
        while remaining>1e-5 and self.elapsed<deadline:
            delta=math.copysign(min(abs(increment),remaining,max(0.,deadline-self.elapsed)*self.turn_speed),angle)
            self.yaw=wrap(self.yaw+delta); self.elapsed+=abs(delta)/self.turn_speed
            self.scan(); remaining-=abs(delta)
            if len(self.found)==3: break

    def hold(self,seconds,deadline):
        end=min(deadline,self.elapsed+seconds)
        while self.elapsed<end and len(self.found)<3:
            self.elapsed=min(end,self.elapsed+self.camera_period)
            self.scan()

    def step(self,weights,deadline=600.):
        self.steps+=1
        choices,previous=self.options(self.map,self.res,(0.,0.),(self.x,self.y),self.yaw,self.visits,self.viewed,True)
        goal=self.choose(choices,weights)
        if goal is None:
            self.visits=self.visits[-5:]; self.rotate(math.pi,deadline)
            if self.camera_period:self.hold(2.,deadline)
            elif len(self.found)<3:self.elapsed=min(deadline,self.elapsed+2.)
            return
        row,col=goal['cell']; width=self.map.shape[1]; cell=row*width+col
        start=int(self.y/self.res)*width+int(self.x/self.res)
        path=[]
        for _ in range(self.map.size):
            if cell==start: break
            if cell<0: return
            path.append(divmod(cell,width)); cell=int(previous[cell])
        else: raise RuntimeError('Path cycle')
        path.reverse()
        # String-pull on OBSERVED free space, approximating Nav2's smoothed path.
        smooth=[]; position=((self.y/self.res-.5,self.x/self.res-.5) if self.continuous_shortcuts
                            else (int(self.y/self.res),int(self.x/self.res))); index=0
        safe=distance_transform_edt(self.map==0)*self.res>=.29
        while index<len(path):
            end=index
            for j in range(index+1,len(path)):
                a,b=path[j]; steps=int(math.ceil(max(abs(a-position[0]),abs(b-position[1]))*3))+1
                ys=np.rint(np.linspace(position[0],a,steps)).astype(int)
                xs=np.rint(np.linspace(position[1],b,steps)).astype(int)
                if not safe[ys[1:],xs[1:]].all(): break
                if self.continuous_shortcuts:
                    from osp_mission.path_geometry import segment_clear
                    if not segment_clear(self.map,self.res,position,(a,b)):break
                end=j
            target=path[end]
            length=math.hypot(target[0]-position[0],target[1]-position[1])*self.res
            for u in np.linspace(0,1,max(2,int(math.ceil(length/(self.speed*self.camera_period if self.camera_period else .25)))+1))[1:]:
                smooth.append((position[0]+u*(target[0]-position[0]),position[1]+u*(target[1]-position[1])))
            position=target; index=end+1
        for row,col in smooth:
            if self.elapsed>=deadline or len(self.found)==3: break
            nx=(col+.5)*self.res; ny=(row+.5)*self.res
            heading=math.atan2(ny-self.y,nx-self.x)
            if self.camera_period:
                self.rotate(wrap(heading-self.yaw),deadline)
                if self.elapsed>=deadline or len(self.found)==3:break
            else:
                self.elapsed+=abs(wrap(heading-self.yaw))/self.turn_speed
                self.yaw=heading; self.scan()
            # Local Nav2-like shield uses only the latest observed scan map.
            # Replan when newly observed geometry blocks a previously clear route.
            if motion_blocked(self.map,self.res,nx,ny,cell=(int(row+.5),int(col+.5))):
                self.safety_stops+=1; self.elapsed+=2.; self.scan(); break
            if self.clearance[int(row+.5),int(col+.5)]<BODY_RADIUS:
                self.collisions+=1; self.elapsed+=8.; break
            d=math.hypot(nx-self.x,ny-self.y)
            if self.camera_period and self.elapsed+d/self.speed>deadline:
                fraction=max(0.,deadline-self.elapsed)*self.speed/max(d,1e-12)
                nx=self.x+fraction*(nx-self.x);ny=self.y+fraction*(ny-self.y);d*=fraction
            self.elapsed+=d/self.speed
            self.distance+=d; self.x,self.y=nx,ny; self.scan()
            if self.planner=='edge_glance' and len(self.found)<3 and self.elapsed<deadline:
                self.glance_during_travel(deadline)
        self.visits.append((self.x,self.y))
        if self.elapsed<deadline and len(self.found)<3:
            scan_yaw=goal['yaw']
            sweep=goal.get('scan_angle',2*math.pi/3) if self.directional_coverage or self.edge_coverage is not None or self.planner in ('inspection','surface','surface_balanced','surface_v2','surface_endpoint','endpoint_faces','endpoint_prior','endpoint_prior_faces','surface_deadline') else scan_angle(weights)
            from osp_mission.scan_alignment import plan
            optimized_yaw,optimized_sweep=plan(self.yaw,scan_yaw,sweep)
            self.avoidable_alignment_radians+=max(0.,abs(wrap(scan_yaw-self.yaw))-abs(wrap(optimized_yaw-self.yaw)))
            if self.optimize_scan_alignment:scan_yaw,sweep=optimized_yaw,optimized_sweep
            before=self.elapsed
            if self.camera_period:
                self.rotate(wrap(scan_yaw-self.yaw),deadline)
                self.observation_alignment_seconds+=self.elapsed-before
                if self.elapsed>=deadline or len(self.found)==3:return
            else:
                self.elapsed+=abs(wrap(scan_yaw-self.yaw))/self.turn_speed
                self.yaw=scan_yaw; self.scan()
                self.observation_alignment_seconds+=self.elapsed-before
            before=self.elapsed;self.rotate(sweep,deadline)
            self.observation_sweep_seconds+=self.elapsed-before
            if self.camera_period:self.hold(1.5,deadline)
            else:self.elapsed+=1.5

    def glance_during_travel(self,deadline):
        if self.distance-self._last_glance_distance<1.5 or self.elapsed-self._last_glance_time<10.:
            return
        self._last_glance_distance=self.distance
        from osp_mission.travel_glance import select
        glance=select(self.map,self.res,(0.,0.),(self.x,self.y,self.yaw),
            self.edge_coverage.faces,self.viewed,self.camera_focal,1.25,self.travel_glance_threshold)
        if glance is None:return
        original_heading=self.yaw
        self.travel_glances+=1;self._last_glance_time=self.elapsed
        self.rotate(wrap(glance['yaw']-self.yaw),deadline)
        self.hold(.8,deadline)
        if len(self.found)<3 and self.elapsed<deadline:
            self.rotate(wrap(original_heading-self.yaw),deadline)

    def run(self,weights,deadline=600.):
        self.deadline=deadline
        self.rotate(2*math.pi,deadline)
        while self.elapsed<deadline and len(self.found)<3 and self.steps<160: self.step(weights,deadline)
        success=len(self.found)==3 and self.elapsed<=deadline
        elapsed=min(self.elapsed,deadline)
        # Completion dominates time; partial discoveries still provide a learning signal.
        reward=100*len(self.found)+300*int(success)-.15*elapsed-100*self.collisions+.02*float((self.map>=0).sum())
        return dict(seed=self.cfg['seed'],difficulty=self.cfg.get('difficulty',0),found=len(self.found),
                    success=success,seconds=float(elapsed),collisions=self.collisions,safety_stops=self.safety_stops,distance=float(self.distance),
                    decisions=self.steps,reward=float(reward),coverage=float((self.map>=0).mean()))


def episode(cfg,weights): return SearchEnv(cfg,seed=cfg['seed']+123456).run(np.asarray(weights))
