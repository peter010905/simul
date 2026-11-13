"""Bounded looks at nearby uninspected faces; no target IDs or scene inputs."""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

from osp_mission.edge_policy import boundary_faces,readable_distance,visible
from osp_mission.policy import wrap


class NearPassInspection:
    maximum_camera_range_m=1.2
    maximum_turn_rad=2*math.pi/3
    minimum_surface_length_m=.25
    maximum_total_sim_s=35.
    maximum_looks=8

    def __init__(self):
        self.history=[];self.spent=0.;self.last_started=-math.inf;self.active=None

    def choose(self,data,resolution,origin,camera,robot,viewed,focal,fov,odom_camera,
               now,elapsed,remaining,protected=False):
        if (protected or self.active is not None or elapsed<15. or remaining<45.
                or len(self.history)>=self.maximum_looks or now-self.last_started<15.):return None
        if not np.isfinite([resolution,*origin,*camera,*robot,focal,fov,*odom_camera,now,elapsed,remaining]).all():return None
        if resolution<=0 or focal<=0 or not 0<fov<math.pi:return None
        if viewed.shape!=(4,)+data.shape:raise ValueError('Four boundary-face states required')
        base=np.floor((np.asarray(robot)-np.asarray(origin))/resolution).astype(int)
        if not (0<=base[1]<data.shape[0] and 0<=base[0]<data.shape[1]):return None
        if data[base[1],base[0]]!=0:return None
        # Nav2 retains ownership of the stationary rotation and collision checks.
        # Known free footprint clearance is an additional precondition.
        clearance=distance_transform_edt(data==0)*resolution
        if clearance[base[1],base[0]]<.32:return None
        faces=boundary_faces(data,resolution)
        if not len(faces['side']):return None
        cy,cx=(camera[1]-origin[1])/resolution,(camera[0]-origin[0])/resolution
        dy,dx=cy-faces['py'],cx-faces['px'];lengths=np.hypot(dy,dx)
        distances=lengths*resolution
        incidence=(dy*faces['ny']+dx*faces['nx'])/np.maximum(lengths,1e-9)
        bearing=(np.arctan2(-dy,-dx)-camera[2]+math.pi)%(2*math.pi)-math.pi
        eligible=~viewed[faces['side'],faces['y'],faces['x']]
        eligible &= (distances>.35)&(distances<=self.maximum_camera_range_m)&(incidence>.6)
        eligible &= distances<readable_distance(focal)*incidence
        eligible &= abs(bearing)>fov/2+.08
        ids=np.flatnonzero(eligible)
        if not len(ids):return None
        ids=ids[visible(data,cy,cx,faces['py'][ids],faces['px'][ids])]
        if not len(ids):return None
        choices=[]
        for turn in np.arange(-self.maximum_turn_rad,self.maximum_turn_rad+1e-9,math.pi/12):
            if abs(turn)<fov/2:continue
            selected=ids[abs((bearing[ids]-turn+math.pi)%(2*math.pi)-math.pi)<fov/2-.12]
            length=len(selected)*resolution
            if length+1e-9<self.minimum_surface_length_m:continue
            odom_yaw=wrap(odom_camera[2]+turn)
            if any(math.dist(odom_camera[:2],old['odom_camera'][:2])<.8
                    and abs(wrap(odom_yaw-old['odom_yaw']))<math.pi/4 for old in self.history):continue
            estimated=2*abs(float(turn))/.85+1.2
            allowance=min(self.maximum_total_sim_s,5.+.08*elapsed)
            if self.spent+estimated>allowance or estimated>remaining-45.:continue
            angular_gain=float(np.sum(resolution*incidence[selected]/distances[selected]))
            choices.append((angular_gain/estimated,dict(turn_rad=float(turn),
                gain_surface_m=float(length),angular_gain_rad=angular_gain,estimated_sim_s=estimated,
                odom_camera=list(odom_camera),odom_yaw=odom_yaw,
                face_count=len(selected),range_min_m=float(distances[selected].min()),
                range_max_m=float(distances[selected].max()))))
        return max(choices,key=lambda pair:pair[0])[1] if choices else None

    def reserve(self,choice,now):
        if self.active is not None:raise RuntimeError('Inspection already active')
        self.active=dict(choice,started=now);self.history.append(self.active)
        self.last_started=now

    def finish(self,now,reason):
        if self.active is None:return
        duration=max(0.,now-self.active['started']);self.spent+=duration
        self.active.update(actual_sim_s=duration,reason=reason);self.active=None

    def report(self):
        return dict(requested=len(self.history),spent_sim_s=self.spent,
            maximum_sim_s=self.maximum_total_sim_s,maximum_looks=self.maximum_looks,
            maximum_camera_range_m=self.maximum_camera_range_m,
            active=self.active is not None,history=self.history,
            ground_truth_input=False,creates_recognition_or_coverage_credit=False)
