"""One navigation owner for Flow v3, anonymous print follow-up, and R32 transit."""
import json
import math
from collections import deque
from pathlib import Path

import cv2
import numpy as np
from geometry_msgs.msg import PointStamped
from rclpy.time import Time
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
from osp_mission import _r32_navigation_core as r32
from osp_mission.r32_drive import R32Drive, grid_from_message, in_map
from osp_mission.edge_policy import visible
from osp_mission.joint_surface import connecting_distances
from osp_mission import dock_geometry
from osp_mission.wall_route import yaw,camera_pose
from osp_mission.thorough_borders import borders
from osp_mission.hybrid_geometry import preview_geometry
from osp_mission.general_surface import GeneralSurfaceFollowup


def publish_print(node, msg, image, *, decoded=(), min_edge_pixels=0.):
    """Use only measured RGB outlines, exposure-time geometry, and synchronized range."""
    cal = preview_geometry(node.info)
    if cal is node.info and (msg.width,msg.height)==(250,250):
        return
    stamp = Time.from_msg(msg.header.stamp)
    outlines = [(u, v, size) for u, v, size in borders(image)
                if size >= min_edge_pixels and not any(
                    cv2.pointPolygonTest(quad.reshape(4, 2), (u, v), False) >= 0
                    for quad in decoded)]
    if not outlines:
        return
    transform=node.tf.lookup_transform(node.p('odom_frame'),msg.header.frame_id,stamp)
    camera=camera_pose(transform.transform)
    if camera is not None and getattr(node,'print_glance',None) is not None:
        t=stamp.nanoseconds/1e9
        glance=node.print_glance.update(t,t,camera,image,cal.k[0],cal.k[2],outlines,None)
        if glance:
            node.glance_pub.publish(String(data=json.dumps(dict(stamp=t,angle=math.radians(glance['yaw_delta_deg'])))))
    for u,v,size in sorted(outlines,key=lambda p:-p[2])[:4]:
        ray = np.array([(u-cal.k[2])/cal.k[0],(v-cal.k[5])/cal.k[4],1.])
        xyz = node.lidar_point(ray,msg.header.frame_id,stamp)
        if xyz is None:
            continue
        origin = transform.transform.translation
        pt = PointStamped()
        pt.header = msg.header
        pt.point.x,pt.point.y,pt.point.z = map(float,xyz)
        p = do_transform_point(pt,transform).point
        node.print_pub.publish(String(data=json.dumps(dict(stamp=stamp.nanoseconds/1e9,
            point=[p.x,p.y],observer=[origin.x,origin.y],size=size))))


class PrintMemory:
    def __init__(self):
        self.tracks = []
        self.used = []
        self.white_used = []
        self.defer_white = False
        self.white_hold_seconds = 90.
        # .50: real runs read no ID from 19 white-leg looks in 600 s. A leg
        # already looked at stays used for the mission; consecutive looks that
        # add no marker pause further white-leg cues.
        self.white_radius_m = 1.0
        self.white_streak_limit = 3
        self.white_pause_s = 150.
        self.white_streak = 0
        self.white_targets_seen = 0
        self.white_block_until = -math.inf

    def white_dispatched(self, now, target_count):
        if target_count > self.white_targets_seen:
            self.white_streak = 0
        self.white_targets_seen = max(self.white_targets_seen, target_count)
        self.white_streak += 1
        if self.white_streak >= self.white_streak_limit:
            self.white_block_until = now + self.white_pause_s
            self.white_streak = 0

    def white_allowed(self, now, target_count):
        if target_count > self.white_targets_seen:
            self.white_targets_seen = target_count
            self.white_streak = 0
            self.white_block_until = -math.inf
        return now >= self.white_block_until

    def observe(self, evidence, now):
        stamp = float(evidence['stamp'])
        point,observer = (np.asarray(evidence[k],dtype=float) for k in ('point','observer'))
        if (point.shape!=(2,) or observer.shape!=(2,) or not np.isfinite([*point,*observer,stamp]).all()
                or not -.1<=now-stamp<=.5 or not .7<=np.linalg.norm(point-observer)<=8):
            return
        self.tracks = [r for r in self.tracks if now-r['stamp']<=45 or now<=r.get('held_until',-math.inf)]
        white_leg = evidence.get('white_leg') is True
        matching = (r for r in self.tracks if r.get('white_leg', False) == white_leg)
        nearest = min(matching,key=lambda r:np.linalg.norm(point-r['point']),default=None)
        if nearest is not None and np.linalg.norm(nearest['point']-point)<min(.30,.14+.025*np.linalg.norm(point-observer)):
            dt = stamp-nearest['stamp']
            if dt<.075:
                return
            nearest['count'] = nearest['count']+1 if dt<=.55 else 1
            nearest.update(point=(nearest['point']+point)/2,observer=observer,stamp=stamp)
        else:
            self.tracks.append(dict(point=point,observer=observer,stamp=stamp,count=1,white_leg=white_leg))
            self.tracks=self.tracks[-48:]
        if white_leg and self.defer_white:
            for row in self.tracks:
                if row.get('white_leg') and now-row['stamp'] <= .6:
                    row['held_until'] = row['stamp']+self.white_hold_seconds

    def choose(self, grid, cost, pose, transform, targets, now, veto, *, white_filter=None, excluded_zone=None):
        if transform is None:
            return None
        self.used=[r for r in self.used if now-r[1]<180]
        white_ok=self.white_allowed(now,len(targets))
        recent=[r for r in self.tracks if r['count']>=3 and 0<=now-r['stamp'] and (now-r['stamp']<=30 or now<=r.get('held_until',-math.inf))
                and not any(np.linalg.norm(r['point']-p)<.75 for p,_ in self.used)
                and not (r.get('white_leg') and (not white_ok or any(
                    np.linalg.norm(r['point']-p)<self.white_radius_m for p,_ in self.white_used)))]
        if not recent:
            return None
        safe, distances = grid.reachable(pose[:2], .32)
        start = grid.index(*pose[:2])
        nav_start = cost.index(*pose[:2])
        nav = r32.connected_nav_cells(cost, pose)
        if not grid.inside(*start) or not cost.inside(*nav_start) or not nav[nav_start]:
            return None
        candidates = []
        max_detour_m = 2.4
        for r in recent:
            point=np.asarray(in_map(('odom',*r['point']),transform))
            if not r.get('white_leg') and excluded_zone is not None and excluded_zone(point):
                continue
            if r.get('white_leg') and (white_filter is None or not white_filter(point)):
                continue
            if any(np.linalg.norm(point-np.asarray(t[:2]))<.40 for t in targets.values()):
                continue
            observer=np.asarray(in_map(('odom',*r['observer']),transform))
            direction=observer-point
            length=np.linalg.norm(direction)
            if length<.1:
                continue
            direction/=length
            robot=np.asarray(pose[:2])
            on_view_side=(robot-point)@direction>0
            for standoff in (1.2,1.45,1.8):
                stay = on_view_side and np.linalg.norm(robot-point)<=1.5
                position=robot if stay else point+standoff*direction
                if not stay and on_view_side and np.linalg.norm(robot-point)-standoff<.25:
                    continue
                cell=grid.index(*position)
                nc = cost.index(*position)
                if (not grid.inside(*cell) or not safe[cell] or not 0 <= distances[cell] <= max_detour_m+1e-8
                        or not cost.inside(*nc) or not nav[nc] or np.linalg.norm(position-point) < 1.e-6):
                    continue
                end=grid.index(*(point+.20*(position-point)/np.linalg.norm(position-point)))
                if not visible(grid.cells,*cell,np.array([end[0]]),np.array([end[1]]))[0]:
                    continue
                angle=math.atan2(*(point-position)[::-1])
                goal=dict(x=float(position[0]),y=float(position[1]),yaw=angle,
                          scan_angle=0.,distance=float(np.linalg.norm(position-robot)),
                          features=np.zeros(5),stationary=bool(stay),print_point=point.tolist(),
                          print_odom=r['point'].tolist(), white_leg=r.get('white_leg', False))
                if veto(goal):
                    continue
                candidates.append((cell, standoff, goal))
        if not candidates:
            return None
        routes = connecting_distances(grid, cost, safe, nav, start,
                                      (cell for cell, _, _ in candidates), max_detour_m+1e-8)
        reachable = [(routes[cell] + abs(stand-1.2), goal, routes[cell])
                     for cell, stand, goal in candidates if cell in routes]
        if not reachable:
            return None
        _, goal, walk = min(reachable, key=lambda row: (row[1].get('white_leg', False), row[0]))
        goal['print_route_m'] = walk
        return goal


class Hybrid:
    def __init__(self):
        from osp_mission.angular_box import AngularBox
        self.box_views = AngularBox()
        self.surface_followup = GeneralSurfaceFollowup()
        self.next_box_probe = 0.
        self.drive=R32Drive()
        self.memory=PrintMemory()
        self.transit=deque(maxlen=32)
        self.drive.transit=[]
        self.seen_boxes=[]
        self.last_box=-math.inf
        self.next_check=0.
        self.next_interrupt=0.

    def has_fresh_print(self, owner):
        stamp = owner.get_clock().now().nanoseconds/1e9
        service = getattr(getattr(self, 'schedule', None), 'continuity', None)
        lap = getattr(getattr(self,'schedule',None),'wall_lap',None)
        protected = ((service is not None and service.protecting(owner.now()-owner.started))
                     or (lap is not None and lap.phase == 'perimeter'))
        return any(row['count'] >= 3 and 0 <= stamp-row['stamp'] <= 1.
                   and not (protected and row.get('white_leg'))
                   for row in self.memory.tracks)

    def should_interrupt(self,owner):
        now=owner.now()
        if now<max(self.next_check,self.next_interrupt):
            return False
        self.next_check=now+1.
        fresh_print = self.has_fresh_print(owner)
        probe_box = now>=self.next_box_probe and now-self.box_views.last_choice>=10.
        if not fresh_print and not probe_box:
            return False
        # Anonymous interruption is optional while the regional survey has
        # safe local work. Named marker and safety signals use their own paths.
        survey = getattr(getattr(self, 'schedule', None), 'joint', None)
        if not fresh_print and survey is not None and survey.can_continue(now-owner.started):
            return False
        if probe_box:
            self.next_box_probe=now+3.
        try:
            pose=owner.tf.lookup_transform(owner.p('map_frame'),owner.p('base_frame'),Time()).transform
            mt=owner.tf.lookup_transform('odom',owner.p('map_frame'),Time()).transform
            m=owner.map
            data=np.asarray(m.data).reshape(m.info.height,m.info.width).copy()
            dock=owner.known_dock_pose(owner.p('map_frame'))
            if dock is not None:
                dock_geometry.mark(data,m.info.resolution,(m.info.origin.position.x,m.info.origin.position.y),dock)
            proposed=self.select(owner,None,data,None,(pose.translation.x,pose.translation.y,yaw(pose.rotation)),
                                 (mt.translation.x,mt.translation.y,yaw(mt.rotation)),print_only=True, box_probe=probe_box)
        except Exception as exc:
            owner.get_logger().debug(f'Anonymous inspection deferred: {exc}')
            return False
        if proposed is None:
            return False
        self.next_interrupt=now+20.
        return True

    def select(self, owner, selected, data, viewed, pose, transform, *, print_only=False, box_probe=False):
        if selected and ('approach_id' in selected or 'focus_id' in selected):
            return selected
        msg=self.drive.costmap
        clock=owner.get_clock().now().nanoseconds/1e9
        if msg is None or msg.header.frame_id!=owner.p('map_frame') or not -.1<=clock-Time.from_msg(msg.header.stamp).nanoseconds/1e9<=3:
            return selected
        if transform is not None:
            tx,ty,a=transform
            ox=tx+math.cos(a)*pose[0]-math.sin(a)*pose[1]
            oy=ty+math.sin(a)*pose[0]+math.cos(a)*pose[1]
            if not self.transit or math.dist((ox,oy),self.transit[-1][:2])>=.4:
                self.transit.append((ox,oy,pose[2]+a))
            self.drive.transit=[(*in_map(('odom',x,y),transform),h-a) for x,y,h in self.transit]
        else:
            self.drive.transit=[]
        flow=owner.navigation_flow
        def veto(goal):
            return ((flow is not None and flow.blocked(goal,transform,owner.now())) or
                    (not goal.get('stationary') and any(math.dist([goal['x'],goal['y']],xy)<.45 for xy in owner.excluded[-8:])))
        grid=grid_from_message(owner.map)
        grid=r32.Grid(data,grid.resolution,grid.origin)
        print_goal=self.memory.choose(grid,grid_from_message(msg),pose,transform,owner.targets,clock,veto)
        if print_goal is None and (not print_only or box_probe):
            box = (self.box_views.choose(grid,grid_from_message(msg),pose,transform,owner.now(),owner.targets,veto)
                   if owner.p('duration')-(owner.now()-owner.started)>=45. else None)
            if box is not None:
                return box
        if print_goal is not None or print_only:
            return print_goal
        # A completed short look can justify at most one safe closer view of
        # the still-unobserved surface; a marker approach always won above.
        looks=getattr(owner.near_pass.policy,'history',())
        glance=looks[-1] if looks else None
        if glance and transform is not None and viewed is not None:
            try:
                glance_map=in_map(('odom',*glance['odom_camera'][:2]),transform)
                evidence=dict(glance,map_camera=glance_map)
                followup=self.surface_followup.choose(
                    grid,grid_from_message(msg),pose,viewed,evidence,len(looks)-1,
                    owner.now(),owner.p('duration')-(owner.now()-owner.started),veto)
                if followup is not None:return followup
            except Exception as exc:
                owner.get_logger().debug(f'General surface follow-up unavailable: {exc}')
        self.seen_boxes=[(p,t) for p,t in self.seen_boxes if owner.now()-t<180.]
        self.drive.inspect_boxes=False
        candidate=self.drive.choose(owner,data,viewed,pose,transform)
        if candidate and candidate.get('box_center') is not None:
            center=candidate['box_center']
            tx,ty,a=transform
            x,y=center
            center_odom=(tx+math.cos(a)*x-math.sin(a)*y,ty+math.sin(a)*x+math.cos(a)*y)
            if any(math.dist(old,center_odom)<.65 for old,_ in self.seen_boxes):
                return selected
            candidate['box_odom']=center_odom
        if candidate and not veto(candidate) and (selected is None or candidate['r32_kind'] in ('corridor','detour','box_look')):
            return candidate
        return selected

    def dispatched(self,owner,goal):
        self.box_views.dispatched(goal, owner.now())
        self.surface_followup.dispatched(goal, owner.now())
        self.drive.dispatched(goal)
        if 'box_odom' in goal:
            self.last_box=owner.now()
            self.seen_boxes.append((goal['box_odom'],self.last_box))
        if 'print_odom' in goal:
            used = self.memory.white_used if goal.get('white_leg') else self.memory.used
            stamp = owner.get_clock().now().nanoseconds/1e9
            used.append((np.asarray(goal['print_odom']), stamp))
            if goal.get('white_leg'):
                self.memory.white_dispatched(stamp, len(getattr(owner,'targets',None) or {}))
        kind=goal.get('r32_kind','print' if 'print_point' in goal else
                      'approach' if 'approach_id' in goal else 'focus' if 'focus_id' in goal else 'flow')
        try:
            with (Path(owner.p('output_dir'))/'hybrid_goals.jsonl').open('a') as f:
                f.write(json.dumps(dict(t=owner.now(),kind=kind,x=goal['x'],y=goal['y'],print_route_m=goal.get('print_route_m'),box_view_scope=goal.get('box_view_scope')))+'\n')
        except OSError as exc:
            owner.get_logger().debug(str(exc))
