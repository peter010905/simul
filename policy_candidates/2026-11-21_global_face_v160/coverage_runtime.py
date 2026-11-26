"""Integrate the exploration scheduler with the original mission's sole motion owner."""
import json
import math
from pathlib import Path

import numpy as np
from rclpy.time import Time
from tf2_ros import TransformException
from osp_mission import _r32_navigation_core as r32
from osp_mission.center_wall import surface_cells
from osp_mission.face_tasks import local_face,face_evidence
from osp_mission.global_coverage import GlobalCoverage,surface_cloud
from osp_mission.path_review import PathReview
from osp_mission.phased_hybrid import PhasedHybrid
from osp_mission.r32_drive import grid_from_message
from osp_mission.wall_route import yaw


class CoverageHybrid(PhasedHybrid):
    def __init__(self,owner):
        super().__init__()
        self.owner=owner
        self.schedule=GlobalCoverage()
        self.schedule.paths=PathReview(owner)
        self.schedule.threshold=owner.p('coverage_face_fraction')
        self.schedule.camera=None
        self.next_box_probe=math.inf
        self.pending_plan=False
        self.owns_choice=False

    def calibrated(self,msg,camera):
        owner=self.owner
        try:
            base=owner.tf.lookup_transform(owner.p('map_frame'),owner.p('base_frame'),Time.from_msg(msg.header.stamp)).transform
        except TransformException:
            return
        if msg.k[0]<=0 or msg.width<=0:return
        angle=yaw(base.rotation); c,s=math.cos(angle),math.sin(angle)
        dx=camera[0]-base.translation.x;dy=camera[1]-base.translation.y
        self.schedule.camera=(float(msg.k[0]),math.atan(msg.width/(2*msg.k[0])),
            owner.p('coverage_marker_width_m'),owner.p('coverage_min_edge_px'),(c*dx+s*dy,-s*dx+c*dy,camera[2]-angle),owner.coverage.optical_plane)

    def select(self,owner,selected,data,viewed,pose,transform,*,print_only=False,box_probe=False,choices=()):
        paths=self.schedule.paths
        self.pending_plan=False
        self.owns_choice=False
        if print_only:
            return super().select(owner,selected,data,viewed,pose,transform,print_only=True,box_probe=False)
        self.schedule.ranked=[]
        proposed=super().select(owner,selected,data,viewed,pose,transform,choices=choices)
        if proposed is not None and any(k in proposed for k in ('approach_id','focus_id','print_point')):
            paths.discard()
            return proposed
        self.owns_choice=True
        now=owner.now()
        reviewed=paths.take(now)
        ranked=self.schedule.ranked
        if reviewed is not None and math.dist(pose[:2],reviewed['pose'])<=.22 and now-reviewed['start']<=8.:
            feasible=[]
            checked_faces={g.get('object_face') for g,_,reason in reviewed['results'] if reason is None}
            failed_faces=set()
            for old,length,reason in reviewed['results']:
                if reason:
                    task=self.schedule.tasks.by_key.get(old.get('object_face'))
                    if task and task.state!='OBSERVED' and task.key not in checked_faces|failed_faces:
                        task.fail(reason,now-owner.started)
                        failed_faces.add(task.key)
                    continue
                # Match a freshly feasible pose and heading. Dispatch exactly the planner-checked endpoint.
                match=next((g for g in ranked if g.get('object_face')==old.get('object_face') and
                    g['family']==old['family'] and math.dist((g['x'],g['y']),(old['x'],old['y']))<=.08
                    and abs(math.remainder(old['yaw']-g['yaw'],2*math.pi))<.14),None)
                if match is not None:
                    if not self.still_safe(old,data,pose,owner,transform):continue
                    feasible.append((bool(match.get('overdue')),self.schedule.utility(match,length),
                                     dict(old,nav2_path_m=length,wall_gain_m=match['wall_gain_m'],object_gain_m=match['object_gain_m'])))
            if feasible:return max(feasible,key=lambda v:v[:2])[2]
        if ranked:
            if paths.round is None:paths.start(ranked,now,pose)
            self.pending_plan=True
        return None

    @staticmethod
    def still_safe(goal,data,pose,owner,transform):
        from osp_mission.adaptive_faces import clearance_grid
        m=grid_from_message(owner.map)
        grid=r32.Grid(data,m.resolution,m.origin)
        cost=grid_from_message(owner.hybrid.drive.costmap)
        xy=(goal['x'],goal['y'])
        cell=grid.index(*xy); nc=cost.index(*xy)
        if not grid.inside(*cell) or not cost.inside(*nc):return False
        if clearance_grid(grid)[cell]<.32 or not r32.connected_nav_cells(cost,pose)[nc]:return False
        if owner.navigation_flow and owner.navigation_flow.blocked(goal,transform,owner.now()):return False
        if any(math.dist(xy,p)<.45 for p in owner.excluded[-8:]):return False
        if 'surface_map' in goal:
            surface=np.asarray(goal['surface_map']); delta=np.asarray(xy)-surface
            return grid.line_clear(np.asarray(xy),surface+.06*delta/max(.01,np.linalg.norm(delta)))
        return True

    def dispatched(self,owner,goal):
        super().dispatched(owner,goal)
        self.next_box_probe=math.inf
        self.write(owner,'dispatch',goal)

    def finish(self,reason):
        owner=self.owner; schedule=self.schedule
        try:
            grid=grid_from_message(owner.map)
            tr=owner.tf.lookup_transform('odom',owner.p('map_frame'),Time()).transform
            base=owner.tf.lookup_transform(owner.p('map_frame'),owner.p('base_frame'),Time()).transform
            tf=(tr.translation.x,tr.translation.y,yaw(tr.rotation))
            viewed=owner.coverage.grid(grid.cells,grid.resolution,grid.origin[:2])
            xy,n,seen,w=surface_cloud(surface_cells(grid,(base.translation.x,base.translation.y)),grid,viewed)
            schedule.surfaces={k:dict(observed_cells=int((seen&mask).sum()),mapped_cells=int(mask.sum()))
                               for k,mask in [('wall',w),('obstacle',~w)]}
            for f in schedule.tasks.by_key.values():
                face_evidence(f,*local_face(f,tf),xy,n,seen,grid.resolution,schedule.threshold)
        except Exception as exc:
            owner.get_logger().warning(f'Final surface refresh unavailable: {exc}')
        schedule.termination=reason
        schedule.paths.discard()
        self.write(owner,'finish',{})

    def write(self,owner,event,goal):
        fields=('family','object_face','surface_map','nav2_path_m','wall_gain_m','object_gain_m','nbv_score','overdue','predicted_object_faces')
        row=dict(event=event,elapsed=owner.now()-owner.started,goal={k:goal[k] for k in fields if k in goal},
                 registered_ids=sorted(owner.targets),readable_cue_pending=bool(owner.readable_cue),
                 anonymous_tracks=len(self.memory.tracks),**self.schedule.report())
        try:
            with (Path(owner.p('output_dir'))/'global_coverage.jsonl').open('a') as f:
                f.write(json.dumps(row,allow_nan=False)+'\n')
        except OSError as exc:owner.get_logger().warning(str(exc))
