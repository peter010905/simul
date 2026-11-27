from osp_mission.observation_sweep import select as select_observation_sweep
from osp_mission.result_io import write_result
from osp_mission.marker_approach import MarkerApproach
from osp_mission.graph_landmarks import finalize_output
from ament_index_python.packages import get_package_share_directory
"""Nonblocking mission state machine; start is explicitly signalled by service."""
import csv
import copy
import json
import math
from pathlib import Path
import subprocess
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import CameraInfo, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from irobot_create_msgs.msg import HazardDetectionVector
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose, Spin, BackUp
from irobot_create_msgs.action import Undock
from slam_toolbox.srv import SerializePoseGraph
from std_srvs.srv import Trigger
from std_msgs.msg import String, Header
from tf2_ros import Buffer, TransformListener
from osp_mission.policy import options, choose, load_policy, scan_angle
from osp_mission import inspection_policy, directional_policy, surface_endpoint_policy, edge_policy
from osp_mission.surface_coverage import SurfaceCoverage
from osp_mission.time_budget import choose as choose_before_deadline
from osp_mission import dock_geometry
from osp_mission.scan_safety import rear_clear
from osp_mission.policy_audit import save as save_policy_audit
from osp_mission.lifecycle_readiness import Nav2Readiness
from osp_mission.drive_readiness import DriveReadiness
from osp_mission.marker_focus import MarkerFocus
from osp_mission.work_rules import marker_task_dispatched
from osp_mission.marker_directed_look import MarkerDirectedLook
from osp_mission.face_directed_look import FaceDirectedLook
from osp_mission.near_start import validate_radius
from osp_mission.mission_outcome import outcome_status
from osp_mission.surface_sweep import SurfaceSweep
from osp_mission.wall_route import penalize_routes
from osp_mission.contact_inspection import ContactInspection
from osp_mission.transit_control import TransitFlow
from osp_mission.exploration_memory import ExplorationMemory
from osp_mission.action_priority import SENSOR_REASONS, priority
from osp_mission.near_pass_runtime import NearPassRuntime
from osp_mission.general_route import inspectable_route


class Mission(Node):
    def __init__(self):
        super().__init__('mission')
        for k,v in {'duration':600.,'expected_marker_count':3,'output_dir':str(Path.home()/'simul/results'),
                    'map_frame':'map','base_frame':'base_link','odom_frame':'odom','scan_topic':'scan',
                    'survey_reference_marker_m':.14,'survey_readable_pixels':18.,'survey_distant_enabled':True,
                    'undock':False,'policy_file':'','planner':'legacy','require_drive_controller':False,'planner_start_ignore_radius_m':0.,'navigation_flow_enabled':True,'allow_mid_route_wall_inspection':False}.items(): self.declare_parameter(k,v)
        self.p=lambda k:self.get_parameter(k).value
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
        self.inspect=self.p('planner')=='inspection'
        self.directional=self.p('planner')=='directional'
        self.edge_mode=self.p('planner')=='edge'
        self.planner_start_ignore_radius_m=validate_radius(self.p('planner_start_ignore_radius_m'))
        self.surface_mode=self.p('planner') in ('directional','surface_endpoint','edge')
        if self.p('planner') not in ('inspection','legacy','directional','surface_endpoint','edge'):raise ValueError('Unknown planner')
        module=edge_policy if self.edge_mode else directional_policy if self.directional else surface_endpoint_policy if self.surface_mode else inspection_policy if self.inspect else None
        self.policy_options=module.options if module else options
        self.policy_choose=module.choose if module else choose
        self.policy_load=module.load_policy if module else load_policy
        self.initial_scan=True;self.next_scan_angle=2*math.pi/3
        self.coverage=edge_policy.EdgeCoverage() if self.edge_mode else SurfaceCoverage(directional=self.directional) if self.surface_mode else None
        self.marker_focus=MarkerFocus()
        self.marker_look=MarkerDirectedLook(self)
        self.face_look=FaceDirectedLook(self)
        self.exploration_memory=ExplorationMemory()
        self.near_pass=NearPassRuntime(self)
        self.wall_look=SurfaceSweep()
        self.contact_inspection=ContactInspection()
        self.visual_ids = set()
        self.readable_cue = None
        self.readable_turn = False
        self.create_subscription(String,'mission/marker_bearing',self.on_marker_bearing,10)
        self.navigation_flow=(TransitFlow(bool(self.p('allow_mid_route_wall_inspection'))) if self.p('navigation_flow_enabled') else None)
        if self.edge_mode:
            self.create_subscription(String,'mission/marker_observations',self.on_marker_hint,10)
            self.create_subscription(String,'mission/marker_candidates',self.on_marker_hint,10)
        self.latest_scan=None;self.last_camera=0.;self.last_camera_stamp=-1
        self.scan_stamp=None;self.camera_stamp=None;self.pending_camera=None
        self.cancel_reason='';self.cancel_started=None;self.undock_diagnostics={};self.last_readiness_reason=''
        self.resume_action=None;self.active_goal=None;self.spin_progress=0.;self.action_generation=0
        self.sensor_interruptions=0
        self.camera_diagnostics={'received':0,'late':0,'invalid_calibration':0,'duplicate':0,'accepted':0,'coverage_updates':0,'tf_waits':0}
        self.weights,self.surface_settings=self.load_configuration(self.p('policy_file') if self.p('policy_file') and Path(self.p('policy_file')).expanduser().exists() else ''); self.policy_mtime=0.; self.view_samples=[]; self.last_view=0.
        self.nav=ActionClient(self,NavigateToPose,'navigate_to_pose')
        self.backup=ActionClient(self,BackUp,'backup'); self.recovery_pending=False; self.spin_blocked_until=0.; self.plan_after=0.; self.last_recovery=-100.; self.rear_clear=False
        self.spin=ActionClient(self,Spin,'spin'); self.undock=ActionClient(self,Undock,'undock')
        self.nav2_state=Nav2Readiness(self)
        self.drive_state=DriveReadiness(self) if self.p('require_drive_controller') else None
        self.wheel_stamp=None
        if self.drive_state is not None:
            self.create_subscription(Odometry,'wheel/odom',self.on_wheel_odom,qos_profile_sensor_data)
        self.serializer=self.create_client(SerializePoseGraph,'slam_toolbox/serialize_map')
        self.stop=self.create_publisher(TwistStamped,'cmd_vel',10)
        self.known_obstacles=self.create_publisher(PointCloud2,'mission/known_obstacles',qos_profile_sensor_data) if self.p('undock') else None
        self.status=self.create_publisher(String,'mission/status',10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid,'map',self.on_map,qos)
        self.create_subscription(MarkerArray,'targets',self.on_targets,10)
        self.collision_count=0; self.last_bump=-100.
        self.create_subscription(HazardDetectionVector,'hazard_detection',self.on_hazard,qos_profile_sensor_data)
        self.create_subscription(LaserScan,self.p('scan_topic'),self.on_scan,qos_profile_sensor_data)
        if self.surface_mode:
            self.create_subscription(CameraInfo,'mission/processed_camera',self.on_camera,qos_profile_sensor_data)
        self.create_service(Trigger,'mission/start',self.start)
        self.create_service(Trigger,'mission/stop',self.abort)
        self.approach=MarkerApproach(self.p('expected_marker_count'))
        self.graph_observations=[]
        self.create_subscription(String,'mission/marker_observations',self.on_approach_image,10)
        self.map=None; self.targets={}; self.last_scan=0.; self.state='idle'
        self.goal_handle=None; self.pending=False; self.excluded=[]; self.action_start=0.; self.scan_after=True
        self.started=0.; self.save_proc=None; self.serialize_future=None; self.save_deadline=0.
        self.policy_decisions=0
        from osp_mission.phased_hybrid import PhasedHybrid as Hybrid
        self.contact_pending=False
        self.hybrid=Hybrid() if self.edge_mode else None
        if self.hybrid is not None:
            self.create_subscription(String, 'mission/distant_marker_hint', self.on_distant_hint, 5)
            self.hybrid.schedule.survey.reference_m = self.p('survey_reference_marker_m')
            self.hybrid.schedule.survey.readable_px = self.p('survey_readable_pixels')
            self.create_subscription(OccupancyGrid,'global_costmap/costmap',self.hybrid.drive.update_costmap,qos_profile_sensor_data)
            self.create_subscription(OccupancyGrid,'local_costmap/costmap',self.hybrid.update_local_cost,qos_profile_sensor_data)
            self.create_subscription(String,'mission/small_wall_bearings',self.on_small_wall,10)
            self.create_subscription(String, 'mission/edge_print_bearing', self.near_pass.receive_print, 5)
            self.create_subscription(String,'mission/anonymous_print',self.on_anonymous_print,10)
            self.create_subscription(String,'mission/anonymous_glance',self.on_anonymous_glance,10)
        self.create_timer(.5,self.tick)

    def on_anonymous_glance(self,msg):
        guard=self.contact_inspection
        now=self.now()
        if (self.state!='running' or self.contact_pending or self.recovery_pending or guard.phase
                or self.marker_look.pending or self.face_look.pending or self.near_pass.protected_navigation
                or (guard.bearing and now-guard.bearing[0]<=2.) or now<self.spin_blocked_until):
            return
        try:
            e=json.loads(msg.data)
            angle=float(e['angle'])
            if abs(angle)<=math.radians(6.01) and -.1<=self.get_clock().now().nanoseconds/1e9-e['stamp']<=.5:
                guard.bearing=(now,angle)
        except (TypeError,ValueError,KeyError):
            return

    def on_small_wall(self, msg):
        if self.state != 'running':
            return
        try:
            cue = json.loads(msg.data)
            now = self.get_clock().now().nanoseconds/1e9
            origin = tuple(map(float, cue['origin']))
            if len(origin) == 2:
                for bearing in cue['bearings'][:8]:
                    angle, size = map(float, bearing[:2])
                    structured = len(bearing) > 2 and bearing[2] is True
                    self.hybrid.schedule.weak.observe(float(cue['stamp']),now,origin,angle,size,structured)
        except (KeyError,ValueError,TypeError):
            return

    def on_anonymous_print(self,msg):
        if self.state=='running':
            try:
                lap = self.hybrid.schedule.wall_lap.phase == 'perimeter'
                self.hybrid.memory.defer_white = lap or self.hybrid.schedule.continuity.protecting(self.now()-self.started)
                self.hybrid.memory.white_hold_seconds = 180. if lap else 90.
                self.hybrid.memory.observe(json.loads(msg.data),self.get_clock().now().nanoseconds/1e9)
            except (ValueError,TypeError,KeyError):
                return

    def observation_turn(self):
        look=getattr(self,'marker_look',None)
        directed=look.consume() if look is not None else None
        if directed is not None:return directed
        face=getattr(self,'face_look',None)
        directed=face.consume() if face is not None else None
        if directed is not None:return directed
        # Older planners keep a bounded inspection; edge planners use camera evidence.
        if not self.edge_mode:
            requested=(2*math.pi/3 if self.initial_scan else self.next_scan_angle) if self.surface_mode or self.inspect else scan_angle(self.weights)
            return math.copysign(min(abs(requested),2*math.pi/3),requested)
        calibration=getattr(self,'inspection_camera',None)
        if self.map is None or calibration is None:return 0.
        try:
            tr=self.tf.lookup_transform(self.p('map_frame'),self.inspection_camera_frame,Time())
            q=tr.transform.rotation
            heading=math.atan2(2*(q.y*q.z-q.w*q.x),2*(q.x*q.z+q.w*q.y))
            m=self.map;data=np.asarray(m.data).reshape(m.info.height,m.info.width)
            origin=(m.info.origin.position.x,m.info.origin.position.y)
            viewed=self.coverage.grid(data,m.info.resolution,origin)
            # Observation coverage still requires exact camera TF and real decoded frames.
            return select_observation_sweep(data,m.info.resolution,origin,
                (tr.transform.translation.x,tr.transform.translation.y,heading),viewed,*calibration)
        except Exception as exc:
            self.get_logger().debug(f'Observation sweep unavailable: {exc}')
            return 0.

    def load_configuration(self,path):
        if self.p('planner')=='surface_endpoint':return surface_endpoint_policy.load_configuration(path)
        return self.policy_load(path),{}

    def now(self):
        return self.get_clock().now().nanoseconds/1e9 if self.get_parameter('use_sim_time').value else time.monotonic()
    def on_marker_bearing(self, m):
        cue=json.loads(m.data)
        clock = self.get_clock().now().nanoseconds/1e9
        if cue.get('source') == 'native_without_range':
            if self.state == 'running' and self.hybrid is not None:
                try:
                    self.hybrid.schedule.bearing_advance.observe(cue, clock)
                except (TypeError, ValueError, KeyError, OverflowError) as exc:
                    self.get_logger().debug(f'Invalid native print bearing: {exc}')
            return
        age=clock-cue['stamp']
        if cue.get('readable') and self.state == 'running' and -.1 <= age <= .5:
            self.visual_ids.add(int(cue['id']))
            if 'yaw_odom' in cue and 'origin_odom' in cue:
                self.readable_cue = cue
            return
        guard=self.contact_inspection
        if (self.state=='running' and not guard.phase and -.1<=age<=.5 and cue['id'] not in self.targets
                and self.now()>=self.spin_blocked_until):
            guard.bearing=(self.now(),cue['angle'])

    def follow_readable_cue(self, now):
        cue = getattr(self, 'readable_cue', None)
        if cue is None:
            return False
        guard = self.contact_inspection
        age = self.get_clock().now().nanoseconds / 1e9 - cue['stamp']
        if (not -.1 <= age <= 3.0 or cue['id'] in self.targets or self.contact_pending
                or self.recovery_pending or guard.phase or guard.pressed or now < self.spin_blocked_until
                or self.near_pass.task or (self.marker_look.pending or {}).get('kind') in ('approach', 'focus')
                or getattr(self, 'batch_approach_id', None) in self.targets):
            self.readable_cue = None
            return False
        busy = self.goal_handle or self.pending
        if busy:
            if self.kind not in ('nav','spin') or guard.action or self.readable_turn:
                return False
            if not self.cancel_reason:
                self.cancel_action('readable_marker')
            return self.cancel_reason == 'readable_marker'
        self.readable_cue = None
        from osp_mission.wall_route import camera_pose, wrap
        try:
            pose = camera_pose(self.tf.lookup_transform('odom', self.inspection_camera_frame, Time()).transform)
        except Exception:
            return False
        if pose is None or math.dist(pose[:2], cue['origin_odom']) > .20:
            return False
        self.face_look.clear('readable_marker')
        self.marker_look.clear('readable_marker')
        self.scan_after = False
        self.resume_action = None
        angle = wrap(cue['yaw_odom'] - pose[2])
        self.get_logger().info(f"Rechecking readable ID {cue['id']}, turn {math.degrees(angle):.1f} deg")
        if abs(angle) < .06:
            self.zero()
            self.plan_after = now + 1.0
        else:
            goal = Spin.Goal()
            goal.target_yaw = max(-.8, min(.8, angle))
            goal.time_allowance.sec = 8
            self.send(self.spin, goal, 'spin')
            self.readable_turn = True
        return True

    def on_hazard(self,m):
        if getattr(self, 'near_pass', None) is not None:
            self.near_pass.print_hazard = bool(m.detections)
            self.near_pass.hazard_received = self.now()
        bump=any(d.type==1 for d in m.detections)
        guard=getattr(self,'contact_inspection',None)
        rising=(guard.hazard([d.header.frame_id for d in m.detections if d.type==1],self.now())
                if guard is not None and self.state=='running' else False)
        if bump and self.state=='running' and (rising or (guard is None and self.now()-self.last_bump>2.)):
            self.collision_count+=1; self.last_bump=self.now()
            # Create 3 owns undocking. Other actions must not keep pushing or
            # start a scan while still in contact with an obstacle.
            if getattr(self,'marker_look',None) is not None:self.marker_look.clear('bump_contact')
            if getattr(self,'face_look',None) is not None:self.face_look.clear('bump_contact')
            if getattr(self,'kind','')=='undock':return
            if getattr(self,'near_pass',None) is not None:self.near_pass.abort('bump_contact')
            self.contact_pending=True;self.resume_action=None
            self.scan_after=False;self.spin_blocked_until=self.now()+30.
            self.recovery_pending=getattr(self,'kind','')!='backup'
            if self.goal_handle or self.pending:self.cancel_action('bump_contact')
            else:self.contact_pending=False
            self.zero()
    def on_map(self,m): self.map=m
    def on_wheel_odom(self,m):
        self.wheel_stamp=Time.from_msg(m.header.stamp).nanoseconds/1e9
    def on_scan(self,m):
        self.last_scan=time.monotonic()
        self.latest_scan=m
        self.scan_stamp=Time.from_msg(m.header.stamp).nanoseconds/1e9
        self.rear_clear=False
        try:
            tr=self.tf.lookup_transform(self.p('base_frame'),m.header.frame_id,Time.from_msg(m.header.stamp))
            q=tr.transform.rotation
            angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            self.rear_clear=rear_clear(m,(tr.transform.translation.x,tr.transform.translation.y,angle))
        except Exception:pass
        if self.surface_mode:return
        if self.last_scan-self.last_view<.8: return
        self.last_view=self.last_scan
        try:
            tr=self.tf.lookup_transform(self.p('map_frame'),m.header.frame_id,Time.from_msg(m.header.stamp))
            base=self.tf.lookup_transform(self.p('map_frame'),self.p('base_frame'),Time())
            q=base.transform.rotation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            q=tr.transform.rotation; angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            for i,r in enumerate(m.ranges):
                a=m.angle_min+i*m.angle_increment+angle
                if .25<r<(2.5 if self.inspect else 3.5) and abs((a-yaw+math.pi)%(2*math.pi)-math.pi)<1.25/2:
                    self.view_samples.append((tr.transform.translation.x+r*math.cos(a),tr.transform.translation.y+r*math.sin(a)))
            self.view_samples=self.view_samples[-150000:]
        except Exception: pass
    def on_camera(self,m):
        self.camera_diagnostics['received']+=1
        stamp=Time.from_msg(m.header.stamp)
        if abs((self.get_clock().now()-stamp).nanoseconds)>750_000_000:
            self.camera_diagnostics['late']+=1;return
        if not np.isfinite(m.k).all() or m.k[0]<=0 or m.width<=0:
            self.camera_diagnostics['invalid_calibration']+=1;return
        seconds=stamp.nanoseconds/1e9
        if self.camera_stamp is not None and seconds<=self.camera_stamp:
            self.camera_diagnostics['duplicate']+=1;return
        # A decoded frame is alive even if its exact-time TF has not arrived yet.
        # Only observe_camera() may credit inspected surfaces.
        self.camera_stamp=seconds;self.pending_camera=m
        self.inspection_camera=(float(m.k[0]),2*math.atan(m.width/(2*m.k[0])))
        self.inspection_camera_frame=m.header.frame_id
        self.camera_diagnostics['accepted']+=1
        self.observe_camera()
    def observe_camera(self):
        m=self.pending_camera
        if m is None or self.map is None or self.latest_scan is None:return
        stamp=Time.from_msg(m.header.stamp);scan=self.latest_scan
        if stamp.nanoseconds<=self.last_camera_stamp:return
        if abs((self.get_clock().now()-stamp).nanoseconds)>750_000_000:return
        if abs((Time.from_msg(scan.header.stamp)-stamp).nanoseconds)>250_000_000:return
        if not np.isfinite(m.k).all() or m.k[0]<=0 or m.width<=0:return
        try:
            tr=self.tf.lookup_transform(self.p('map_frame'),scan.header.frame_id,Time.from_msg(scan.header.stamp))
            camera=self.tf.lookup_transform(self.p('map_frame'),m.header.frame_id,stamp)
            q=tr.transform.rotation;angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            q=camera.transform.rotation
            # Optical camera +Z is the viewing axis; +X is image-right.
            dx=2*(q.x*q.z+q.w*q.y);dy=2*(q.y*q.z-q.w*q.x)
            if math.hypot(dx,dy)<.5:return
            yaw=math.atan2(dy,dx);ranges=np.asarray(scan.ranges)
            self.wall_camera_sample = (stamp.nanoseconds/1e9, camera.transform.translation.x,
                                       camera.transform.translation.y, yaw)
            angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment+angle
            valid=np.isfinite(ranges)&(ranges>max(.25,scan.range_min))&(ranges<scan.range_max)
            endpoints=np.column_stack((tr.transform.translation.x+ranges[valid]*np.cos(angles[valid]),
                                       tr.transform.translation.y+ranges[valid]*np.sin(angles[valid])))
            grid=self.map;data=np.asarray(grid.data).reshape(grid.info.height,grid.info.width)
            self.coverage.observe(data,grid.info.resolution,(grid.info.origin.position.x,grid.info.origin.position.y),
                (camera.transform.translation.x,camera.transform.translation.y,yaw),endpoints,
                stamp.nanoseconds/1e9,m.k[0],2*math.atan(m.width/(2*m.k[0])))
            self.last_camera=time.monotonic()
            self.last_camera_stamp=stamp.nanoseconds
            self.camera_diagnostics['coverage_updates']+=1
            self.pending_camera=None
            if getattr(self,'hybrid',None) is not None:
                views=self.hybrid.box_views
                exposure=stamp.nanoseconds/1e9
                if exposure-views.last_frame>=.4:
                    from osp_mission.r32_drive import grid_from_message
                    from osp_mission.wall_route import yaw as map_yaw
                    mt=self.tf.lookup_transform('odom',self.p('map_frame'),stamp).transform
                    views.observe(grid_from_message(grid),(camera.transform.translation.x,camera.transform.translation.y,yaw),
                                  endpoints,(mt.translation.x,mt.translation.y,map_yaw(mt.rotation)),
                                  exposure,2*math.atan(m.width/(2*m.k[0])))
                    self.camera_diagnostics['two_view_frames']=self.camera_diagnostics.get('two_view_frames',0)+1
                    self.camera_diagnostics['confirmed_boxes']=sum(t.confirmed for t in views.tracks)
                    self.camera_diagnostics['observed_box_faces']=sum(len(t.seen) for t in views.tracks if t.confirmed)

        except Exception as exc:
            self.camera_diagnostics['tf_waits']+=1;self.get_logger().debug(str(exc))
    def on_approach_image(self,m):
        if self.state!='running':return
        try:
            evidence=json.loads(m.data)
            self.graph_observations.append({key:evidence.get(key) for key in
                ('id','stamp','odom_point','min_edge_pixels','provisional')})
            del self.graph_observations[:-30000]
            stamp=float(evidence['stamp'])
            tr=self.tf.lookup_transform(self.p('map_frame'),self.p('base_frame'),Time(seconds=stamp))
            odom_robot=None
            try:
                odom=self.tf.lookup_transform(self.p('odom_frame'),self.p('base_frame'),Time(seconds=stamp)).transform.translation
                odom_robot=(odom.x,odom.y)
            except Exception:pass
            self.approach.observe(evidence,(tr.transform.translation.x,tr.transform.translation.y),
                self.get_clock().now().nanoseconds/1e9,self.p('map_frame'),
                confirmed=int(evidence['id']) in self.targets,odom_robot=odom_robot)
        except Exception as exc:self.get_logger().debug(f'Approach observation unavailable: {exc}')

    def on_targets(self,m):
        if self.state in ('idle','saving','done'): return
        previous = set(self.targets)
        self.targets={a.id:(a.pose.position.x,a.pose.position.y) for a in m.markers if a.ns=='point'}
        if getattr(self,'approach',None) is not None:self.approach.reconcile_targets(self.targets)
        self.registered_pending = ((getattr(self, 'registered_pending', set()) | (set(self.targets) - previous))
                                   & set(self.targets))

    def on_distant_hint(self, msg):
        if self.state != 'running' or self.hybrid is None or not self.p('survey_distant_enabled'):
            return
        try:
            self.hybrid.schedule.survey.printed(json.loads(msg.data),
                self.get_clock().now().nanoseconds/1.e9, self.now()-self.started)
        except (ValueError, TypeError) as exc:
            self.get_logger().debug(f'Invalid distant marker cue: {exc}')

    def on_marker_hint(self,m):
        try:
            evidence=json.loads(m.data)
            observer=evidence.get('camera_origin_map')
            if observer is None:
                tr=self.tf.lookup_transform(self.p('map_frame'),self.p('base_frame'),Time(seconds=float(evidence['stamp'])))
                observer=(tr.transform.translation.x,tr.transform.translation.y)
            if not evidence.get('provisional',False):
                ident=int(evidence['id']); stamp=float(evidence['stamp'])
                point=np.asarray(evidence['map_point'][:2],dtype=float)
                seeds=getattr(self,'marker_hint_seeds',{})
                previous=seeds.get(ident)
                if previous is not None and stamp<=previous[0]:return
                seeds[ident]=(stamp,point);self.marker_hint_seeds=seeds
                if previous is None or stamp-previous[0]>1. or np.linalg.norm(point-previous[1])>.3:return
            accepted=self.marker_focus.observe(evidence,observer[:2],self.get_clock().now().nanoseconds/1e9,self.p('map_frame'))
            if not accepted:
                return
            ident=int(evidence['id'])
            if evidence.get('provisional') and self.state == 'running':
                self.get_logger().info(f"Ranged marker candidate {ident}, {evidence.get('min_edge_pixels')} px, queued for a closer view")
            Mission.interrupt_for_hint(self, (ident,), deferred=False)
        except (KeyError,TypeError,ValueError):
            return
        except Exception as exc:
            self.get_logger().debug(f'Marker follow-up unavailable: {exc}')

    def interrupt_for_hint(self, identities, *, deferred):
        if (self.state != 'running' or getattr(self, 'kind', '') != 'nav'
                or not (self.goal_handle or self.pending) or self.cancel_reason
                or getattr(self, 'recovery_pending', False) or getattr(self, 'contact_pending', False)):
            return False
        guard = getattr(self, 'contact_inspection', None)
        look = getattr(getattr(self, 'marker_look', None), 'pending', None)
        if ((guard and (guard.phase or guard.pressed or guard.action))
                or (look and look.get('kind') in ('focus', 'approach'))
                or getattr(self, 'batch_approach_id', None) in self.targets):
            return False
        clock = self.get_clock().now().nanoseconds/1e9
        for ident in identities:
            if self.marker_focus.request_interrupt(ident, self.targets, clock, deferred=deferred):
                self.get_logger().info(f"Pausing navigation for marker {ident}, deferred={deferred}")
                self.cancel_action('marker_hint')
                self.zero()
                return True
        return False

    def sensor_fresh(self,stamp):
        # ROS time advances with physics in simulation; wall time would falsely
        # cancel actions at low real-time factors. Hardware uses ROS system time.
        return stamp is not None and -.1<=self.get_clock().now().nanoseconds/1e9-stamp<=2.
    def readiness_reason(self):
        if self.map is None:return 'map_unavailable'
        if not self.sensor_fresh(self.scan_stamp):return 'scan_stale'
        if not self.nav.server_is_ready() or not self.spin.server_is_ready():return 'nav2_unavailable'
        if not self.nav2_state.ready():return 'nav2_unavailable'
        if getattr(self,'drive_state',None) is not None:
            if not self.drive_state.ready() or not self.sensor_fresh(self.wheel_stamp):return 'drive_unavailable'
        if self.surface_mode and not self.sensor_fresh(self.camera_stamp):return 'processed_camera_stale'
        try:
            self.tf.lookup_transform(self.p('map_frame'),self.p('base_frame'),Time(),timeout=Duration(seconds=.05))
            return ''
        except Exception:return 'map_to_base_tf_unavailable'
    def ready(self):
        self.last_readiness_reason=self.readiness_reason()
        return not self.last_readiness_reason
    def start(self,req,res):
        if self.state!='idle': res.message='Mission already started; restart launch for another run'; return res
        if not self.ready(): res.message='Waiting for map, fresh scan/camera, TF and Nav2'; return res
        if self.p('undock') and not self.undock.server_is_ready(): res.message='Undock action unavailable'; return res
        self.start_sweep_remaining = math.tau
        self.start_sweep_attempts = 0
        self.started=self.now(); self.state='running'; self.targets={}; res.success=True; res.message='Mission started'
        if self.p('undock'): self.send(self.undock,Undock.Goal(),'undock')
        return res
    def abort(self,req,res):
        if self.state!='running': res.message='No running mission'; return res
        self.finish('operator_stop'); res.success=True; return res
    def zero(self):
        m=TwistStamped(); m.header.stamp=self.get_clock().now().to_msg(); m.header.frame_id=self.p('base_frame'); self.stop.publish(m)
    def known_dock_pose(self,frame):
        if getattr(self,'known_obstacles',None) is None or self.undock_diagnostics.get('status')!=4:return None
        try:
            t=self.tf.lookup_transform(frame,'std_dock_link',Time())
            q=t.transform.rotation
            a=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            return t.transform.translation.x,t.transform.translation.y,a
        except Exception:return None
    def open_escape(self, step=.6, need=.9, half_width=math.radians(15), every=20.):
        """.59: send a Nav2 goal `step` m toward the scan direction with the most free range
        (at least `need` m within +-15 deg). Nav2 plans and collision-checks the motion."""
        if self.now()-getattr(self, 'last_open_escape', -100.) < every:
            return False
        scan = getattr(self, 'latest_scan', None)
        if scan is None:
            return False
        try:
            ts = self.tf.lookup_transform(self.p('base_frame'), scan.header.frame_id, Time())
            tm = self.tf.lookup_transform(self.p('map_frame'), self.p('base_frame'), Time())
        except Exception:
            return False
        def yaw_of(q):
            return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        ranges = np.asarray(scan.ranges, dtype=float)
        ranges = np.where(np.isinf(ranges) & (ranges > 0), scan.range_max, ranges)
        ranges = np.where(np.isfinite(ranges) & (ranges >= scan.range_min), ranges, np.nan)
        angles = scan.angle_min + np.arange(len(ranges))*scan.angle_increment + yaw_of(ts.transform.rotation)
        best = None
        for centre in np.linspace(-math.pi, math.pi, 36, endpoint=False):
            sector = np.abs(np.arctan2(np.sin(angles-centre), np.cos(angles-centre))) < half_width
            values = ranges[sector]
            if not sector.any() or np.isnan(values).all():
                continue
            free = float(np.nanmin(values))
            if best is None or free > best[0]:
                best = (free, float(centre))
        if best is None or best[0] < need:
            return False
        heading = yaw_of(tm.transform.rotation) + best[1]
        x = tm.transform.translation.x + step*math.cos(heading)
        y = tm.transform.translation.y + step*math.sin(heading)
        self.last_open_escape = self.now()
        self.get_logger().warning(f'No reachable goal and rear blocked; stepping {step:.1f} m toward open direction ({best[0]:.1f} m free)')
        g = NavigateToPose.Goal(); g.pose.header.frame_id = self.p('map_frame'); g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = x; g.pose.pose.position.y = y
        g.pose.pose.orientation.z = math.sin(heading/2); g.pose.pose.orientation.w = math.cos(heading/2)
        self.send(self.nav, g, 'nav')
        return True
    def publish_known_obstacles(self):
        if self.known_dock_pose(self.p('map_frame')) is None:return
        header=Header(frame_id='std_dock_link',stamp=self.get_clock().now().to_msg())
        self.known_obstacles.publish(point_cloud2.create_cloud_xyz32(header,dock_geometry.points()))
    def send(self,client,goal,kind):
        self.readable_turn = False
        self.pending=True; self.action_start=self.now(); self.kind=kind;self.cancel_reason='';self.cancel_started=None
        self.active_goal=copy.deepcopy(goal);self.spin_progress=0.;self.action_generation+=1
        generation=self.action_generation
        flow=getattr(self,'navigation_flow',None)
        if flow is not None and kind=='nav':flow.begin_navigation()
        feedback={'feedback_callback':lambda m:self.spin_feedback(m,generation)} if kind in ('spin','startup_spin') else {}
        client.send_goal_async(goal,**feedback).add_done_callback(lambda result:self.accepted(result,generation))
    def spin_feedback(self,message,generation):
        if generation!=self.action_generation or self.kind not in ('spin','startup_spin'):return
        distance=float(message.feedback.angular_distance_traveled)
        if math.isfinite(distance):self.spin_progress=distance
    def cancel_action(self,reason):
        previous = self.cancel_reason
        if priority(reason) <= priority(previous):
            return
        self.cancel_reason = reason
        lap = getattr(getattr(getattr(self,'hybrid',None),'schedule',None),'wall_lap',None)
        if lap is not None:
            lap.interrupted()
        schedule = getattr(getattr(self, 'hybrid', None), 'schedule', None)
        if schedule is not None:
            schedule.region_transit.interrupt(reason)
        self.get_logger().warning(f'Cancel {self.kind}: {reason}')
        if not previous:
            self.cancel_started = self.now()
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
    def accepted(self,future,generation=None):
        if generation is not None and generation != self.action_generation:
            try:
                old_handle = future.result()
                if old_handle.accepted:
                    old_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().debug(f'Obsolete action response: {exc}')
            return
        self.pending=False
        guard=getattr(self,'contact_inspection',None)
        try: handle=future.result()
        except Exception as exc:
            if guard is not None and guard.action:guard.result(False,self.now())
            self.contact_pending=False
            self.get_logger().error(str(exc))
            if getattr(self,'marker_look',None) is not None:self.marker_look.clear('action_request_error')
            if getattr(self,'face_look',None) is not None:self.face_look.clear('action_request_error')
            if self.kind=='undock':self.finish('undock_request_failed')
            return
        if not handle.accepted:
            if guard is not None and guard.action:
                guard.result(False,self.now());self.contact_pending=False;return
            self.contact_pending=False
            if getattr(self,'marker_look',None) is not None:self.marker_look.clear('action_rejected')
            if getattr(self,'face_look',None) is not None:self.face_look.clear('action_rejected')
            if getattr(self,'near_pass',None) is not None and self.near_pass.rejected(self.kind):return
            if self.kind=='nav':
                # NavigationFlow owns timed failure exclusions.
                flow=getattr(self,'navigation_flow',None)
                if flow is not None:flow.record_navigation_result(6,'action_rejected',self.now())
                schedule = getattr(getattr(self, 'hybrid', None), 'schedule', None)
                if schedule is not None:
                    schedule.region_transit.navigation_finished()
            if self.kind in ('spin','startup_spin'):
                self.spin_blocked_until=self.now()+30.; self.scan_after=False; self.recovery_pending=True
            if self.kind=='undock': self.finish('undock_rejected')
            return
        self.goal_handle=handle
        if self.state!='running' or self.cancel_reason: handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(lambda result:self.completed(result,generation))
    def completed(self,future,generation=None):
        if generation is not None and generation != self.action_generation:
            return
        self.goal_handle=None
        self.cancel_started=None
        try: result=future.result()
        except Exception as exc:
            guard=getattr(self,'contact_inspection',None)
            if guard is not None and guard.action:guard.result(False,self.now())
            if getattr(self,'marker_look',None) is not None:self.marker_look.clear('action_result_error')
            if getattr(self,'face_look',None) is not None:self.face_look.clear('action_result_error')
            if self.kind=='undock':
                self.undock_diagnostics={'error':str(exc),'cancel_reason':self.cancel_reason}
                self.finish('undock_result_error')
            return
        if self.state!='running': return
        schedule = getattr(getattr(self, 'hybrid', None), 'schedule', None)
        if schedule is not None:
            schedule.face_result(self.kind, result.status, self.cancel_reason, self.now()-self.started)
        guard=getattr(self,'contact_inspection',None)
        if guard is not None and guard.action:
            self.contact_pending=False
            guard.result(result.status==4,self.now())
            self.scan_after=False
            self.get_logger().info(f'Contact action {self.kind}: status={result.status} next={guard.phase or "replan"}')
            return
        if guard is not None and self.kind=='nav' and result.status==6 and self.now()-self.action_start<.6:
            self.plan_after=self.now()+2.
        if getattr(self,'contact_pending',False):
            if getattr(self,'marker_look',None) is not None:self.marker_look.clear('bump_contact')
            if getattr(self,'face_look',None) is not None:self.face_look.clear('bump_contact')
            self.contact_pending=False;self.resume_action=None;self.scan_after=False
            self.recovery_pending=self.kind!='backup';self.plan_after=self.now()+.5
            if self.kind=='nav':
                # NavigationFlow owns timed failure exclusions.
                flow=getattr(self,'navigation_flow',None)
                if flow is not None:flow.record_navigation_result(result.status,'bump_contact',self.now())
            return
        if self.kind == 'startup_spin':
            travelled = abs(self.spin_progress)
            remaining = max(0., self.active_goal.target_yaw-travelled)
            if result.status == 5 and self.cancel_reason in SENSOR_REASONS and self.start_sweep_attempts < 2 and remaining > .04:
                self.start_sweep_remaining = remaining
            elif result.status != 4:
                self.spin_blocked_until = self.now()+30.
            self.get_logger().info(f'Startup sweep status={result.status} feedback={travelled:.3f} rad remaining={remaining:.3f}')
            return
        if self.cancel_reason == 'inspection_due' and result.status == 5:
            self.scan_after = True
            return
        if self.cancel_reason == 'readable_marker' and result.status in (4,5):
            self.face_look.clear('readable_marker')
            self.scan_after = False
            self.plan_after = self.now()
            return
        if self.cancel_reason == 'registered_marker' and self.kind in ('nav','spin') and result.status in (4,5):
            self.marker_look.clear('registered_marker')
            self.face_look.clear('registered_marker')
            self.scan_after = False
            self.plan_after = self.now()
            return
        if getattr(self,'near_pass',None) is not None and self.near_pass.completed(result.status,self.kind,getattr(self,'cancel_reason','')):return
        if result.status==5 and getattr(self,'cancel_reason','') in SENSOR_REASONS and self.kind in ('nav','spin'):
            goal=copy.deepcopy(self.active_goal)
            if self.kind=='spin':
                goal.target_yaw=math.copysign(max(0.,abs(goal.target_yaw)-abs(self.spin_progress)),goal.target_yaw)
            self.resume_action=(self.kind,goal);self.sensor_interruptions+=1
            self.get_logger().warning(f'{self.kind} paused for {self.cancel_reason}; retaining the unfinished goal')
            return
        if result.status==5 and getattr(self,'cancel_reason','')=='marker_hint' and self.kind=='nav':
            if getattr(self,'marker_look',None) is not None:self.marker_look.clear('new_marker_hint')
            if getattr(self,'face_look',None) is not None:self.face_look.clear('new_marker_hint')
            # Hold long enough for already-readable RGB frames to confirm. If
            # they do not, the next planning pass can choose a closer view.
            self.scan_after=False;self.plan_after=self.now()+1.;return
        if self.kind=='nav':
            flow=getattr(self,'navigation_flow',None)
            if flow is not None:flow.record_navigation_result(result.status,getattr(self,'cancel_reason',''),self.now())
            schedule = getattr(getattr(self, 'hybrid', None), 'schedule', None)
            if schedule is not None:
                schedule.box_recovery.navigation_result(result.status, self.cancel_reason, self.now()-self.started)
                schedule.region_transit.navigation_finished()
            if getattr(self,'exploration_memory',None) is not None:self.exploration_memory.completed(result.status==4,self.now())
            lap = getattr(schedule,'wall_lap',None)
            # Successful observations are remembered with orientation, never banned.
            self.scan_after = result.status==4 and not (lap is not None and lap.last_was_lap)
            if result.status!=4 and getattr(self,'marker_look',None) is not None:self.marker_look.clear('navigation_failed')
            if result.status!=4 and getattr(self,'face_look',None) is not None:self.face_look.clear('navigation_failed')
        elif self.kind=='spin' and result.status!=4:
            self.spin_blocked_until=self.now()+30.; self.scan_after=False; self.recovery_pending=True
            self.get_logger().warning('Spin blocked; suppressing retries for 30 s and checking a safe retreat')
        elif self.kind=='spin':
            flow=getattr(self,'navigation_flow',None)
            if flow is None:self.plan_after=self.now()+1.5
            else:
                flow.begin_settle(self.now(),self.camera_diagnostics['accepted']);self.plan_after=self.now()
        elif self.kind=='backup':
            flow=getattr(self,'navigation_flow',None)
            if flow is not None:flow.backup_result(result.status,self.now())
            self.scan_after=False; self.plan_after=self.now()+1.
        elif self.kind=='undock':
            self.undock_diagnostics={'status':int(result.status),'is_docked':bool(result.result.is_docked),
                'cancel_reason':self.cancel_reason,'elapsed_seconds':self.now()-self.action_start}
            self.get_logger().info(f'Undock result: {self.undock_diagnostics}')
            if result.status!=4 or result.result.is_docked:self.finish('undock_failed')
    def finish(self,reason):
        if self.state!='running': return
        if getattr(self,'marker_look',None) is not None:self.marker_look.clear('mission_finished')
        if getattr(self,'face_look',None) is not None:self.face_look.clear('mission_finished')
        if getattr(self,'near_pass',None) is not None:self.near_pass.abort('mission_finished')
        self.reason=reason; self.elapsed=self.now()-self.started; self.state='saving'
        if self.goal_handle: self.goal_handle.cancel_goal_async()
        self.zero()
        self.output=Path(self.p('output_dir')).expanduser()/time.strftime('%Y%m%d_%H%M%S')
        self.output.mkdir(parents=True,exist_ok=False)
        with (self.output/'markers.csv').open('w') as f:
            w=csv.writer(f); w.writerow(['id','x','y'])
            for ident,(x,y) in sorted(self.targets.items()): w.writerow([ident,x,y])
        # Keep the executor alive while map saver and serialization run.
        self.save_log=(self.output/'map_saver.log').open('w')
        self.save_proc=subprocess.Popen(['ros2','run','nav2_map_server','map_saver_cli','-f',str(self.output/'map'),'--free','0.196','--occ','0.65',
            '--ros-args','-p','save_map_timeout:=15.0','-p','free_thresh_default:=0.196','-p','occupied_thresh_default:=0.65',
            '-p','use_sim_time:='+str(self.get_parameter('use_sim_time').value).lower(),'-r','__ns:='+self.get_namespace()],stdout=self.save_log,stderr=subprocess.STDOUT)
        self.save_deadline=time.monotonic()+25
    def save_tick(self):
        self.zero()
        if self.serialize_future is None and self.serializer.service_is_ready():
            req=SerializePoseGraph.Request(); req.filename=str(self.output/'map')
            self.serialize_future=self.serializer.call_async(req)
        if time.monotonic()<self.save_deadline and (self.save_proc.poll() is None or self.serialize_future is None or not self.serialize_future.done()): return
        if self.save_proc.poll() is None: self.save_proc.terminate()
        self.save_log.close()
        self.graph_landmark_update=dict(applied=False,ground_truth_inputs=False,
            corrected_ids=[],reason='SLAM_serialization_not_completed')
        try:
            if self.serialize_future and self.serialize_future.done() and self.serialize_future.result().result==0:
                exporter=Path(get_package_share_directory('osp_mission'))/'tools/export_slam_graph_poses'
                self.targets,self.graph_landmark_update=finalize_output(
                    self.graph_observations,self.targets,self.output,exporter)
        except Exception as exc:
            self.graph_landmark_update.update(error=repr(exc),original_coordinates_preserved=True)
        files=['map.pgm','map.yaml','map.posegraph','map.data','markers.csv']
        report={'face_directed_look':self.face_look.report(),'marker_directed_look':self.marker_look.report(),'graph_landmark_update':self.graph_landmark_update,'near_pass_inspection':self.near_pass.policy.report(),'marker_focus_profile':dict(
                    desired_edge_pixels=getattr(getattr(self,'marker_focus',None),'desired_edge_pixels',None),
                    uncertain_view_penalty_s=getattr(getattr(self,'marker_focus',None),'uncertain_view_penalty_s',None)),
                'approach':self.approach.report(),'reason':self.reason,'elapsed_seconds':self.elapsed,'targets':len(self.targets),'visual_ids':sorted(getattr(self,'visual_ids',())),'collisions':self.collision_count,
                'files':{p:(self.output/p).exists() for p in files},'map_saver_exit':self.save_proc.poll(),
                'undock':self.undock_diagnostics,'last_readiness_reason':self.last_readiness_reason,
                'sensor_interruptions':self.sensor_interruptions,'camera_diagnostics':self.camera_diagnostics}
        try: report['serialize_result']=self.serialize_future.result().result if self.serialize_future and self.serialize_future.done() else 'timeout'
        except Exception as exc: report['serialize_result']=str(exc)
        write_result(self.output/'result.json',report); self.state='done'
        self.get_logger().info(f'Results: {self.output}: {report}')
    def tick(self):
        if getattr(self,'nav2_state',None) is not None:self.nav2_state.poll()
        if getattr(self,'drive_state',None) is not None:self.drive_state.poll()
        lap_status = getattr(getattr(getattr(self,'hybrid',None),'schedule',None),'wall_lap',None)
        if lap_status is not None and self.state == 'running':
            lap_status.expire(self.now()-self.started,self.p('duration'))
        self.status.publish(String(data=json.dumps({'wall_lap':lap_status.report() if lap_status else None,'state':self.state,'approached_ids':self.approach.report()['verified_ids'] if getattr(self,'approach',None) is not None else [],'targets':len(self.targets),'visual_ids':sorted(getattr(self,'visual_ids',())),'collisions':self.collision_count,
            **outcome_status(self),
            'active_action':getattr(self,'kind','') if self.goal_handle or self.pending else '',
            'readiness_reason':getattr(self,'last_readiness_reason',''),
            'sensor_interruptions':getattr(self,'sensor_interruptions',0),
            'planner_start_ignore_radius_m':getattr(self,'planner_start_ignore_radius_m',0.),
            'nav2_lifecycle':self.nav2_state.states if getattr(self,'nav2_state',None) is not None else {},
            'drive_controllers':self.drive_state.states if getattr(self,'drive_state',None) is not None else {},
            'confirmed_surface_cells':int(np.count_nonzero(self.coverage.viewed)) if getattr(self,'coverage',None) is not None and self.coverage.viewed is not None else 0,
            'confirmed_face_anchors':len(getattr(getattr(self,'coverage',None),'anchors',())),
            'reassociated_face_records':int(getattr(getattr(self,'coverage',None),'reassociated_faces',0)),
            'camera_diagnostics':getattr(self,'camera_diagnostics',{}),
            'navigation_flow':self.navigation_flow.report() if getattr(self,'navigation_flow',None) is not None else {}})))
        if self.state=='saving': self.save_tick(); return
        if self.state=='done': self.zero(); return
        if self.state!='running': return
        try:
            pose=self.tf.lookup_transform('odom',self.p('base_frame'),Time()).transform
            visit=pose.translation
            if hasattr(self,'contact_inspection'):
                self.contact_inspection.position=(visit.x,visit.y)
            self.exploration_memory.observe(visit.x,visit.y,self.now())
            if self.hybrid is not None:
                elapsed = self.now() - self.started
                self.hybrid.schedule.survey.position(visit.x, visit.y, elapsed)
                self.hybrid.schedule.wall_lap.position(visit.x, visit.y, elapsed)
                self.hybrid.schedule.joint.position(visit.x, visit.y, elapsed)
            flow=getattr(self,'navigation_flow',None)
            if flow is not None and getattr(self,'kind','')=='nav' and (self.goal_handle or self.pending):
                flow.observe_position(visit.x,visit.y,self.now())
                if isinstance(flow, TransitFlow):
                    flow.observe_heading(pose.rotation, self.now())
        except Exception:pass
        if getattr(self,'approach',None) is not None and self.targets:
            try:
                pose=self.tf.lookup_transform(self.p('map_frame'),self.p('base_frame'),Time())
                self.approach.observe_pose(self.targets,(pose.transform.translation.x,pose.transform.translation.y),
                    Time.from_msg(pose.header.stamp).nanoseconds/1e9,self.get_clock().now().nanoseconds/1e9)
            except Exception as exc:
                self.get_logger().debug(f'Approach pose unavailable: {exc}')
        if getattr(self,'known_obstacles',None) is not None:self.publish_known_obstacles()
        if getattr(self,'approach',None) is not None and self.approach.complete(self.targets): self.finish('all_targets_found'); return
        if self.now()-self.started>=self.p('duration'): self.finish('timeout'); return
        if (self.pending or self.goal_handle) and self.cancel_started is not None:
            if self.now()-self.cancel_started >= 8.:
                self.zero()
                # .52: a late cancel response (slow simulator or loaded robot computer)
                # abandons the stale action instead of ending the mission. Three such
                # events within 90 s still stop the mission as before.
                late = [t for t in getattr(self, 'late_cancels', []) if self.now()-t < 90.] + [self.now()]
                self.late_cancels = late
                if len(late) >= 3:
                    self.finish('action_cancel_unconfirmed')
                    return
                self.get_logger().warning(f'Cancel of {self.kind} unconfirmed after 8 s; abandoning it')
                self.action_generation += 1
                self.goal_handle = None
                self.pending = False
                self.cancel_started = None
                self.cancel_reason = ''
                flow = getattr(self, 'navigation_flow', None)
                if flow is not None and getattr(self, 'kind', '') == 'nav':
                    flow.record_navigation_result(6, 'cancel_unconfirmed', self.now())
                return
        # .53: no progress while driving. If the robot moves less than 0.10 m over 10 s
        # while navigation goals are active (goals may be replaced meanwhile), cancel and
        # run the existing scan-checked backup at once. Stationary looks and spins reset it.
        if getattr(self, 'kind', '') == 'nav' and (self.goal_handle or self.pending) and not self.recovery_pending:
            try:
                tr = self.tf.lookup_transform(self.p('map_frame'), self.p('base_frame'), Time())
                xy = (tr.transform.translation.x, tr.transform.translation.y)
            except Exception:
                xy = None
            if xy is not None:
                now = self.now()
                hist = [h for h in getattr(self, 'stuck_history', []) if now-h[0] <= 10.5] + [(now, xy)]
                self.stuck_history = hist
                if (now-hist[0][0] >= 10. and max(math.dist(hist[0][1], h[1]) for h in hist) < .10
                        and now-getattr(self, 'last_unstick', -100.) >= 12.):
                    self.last_unstick = now
                    self.unstick_events = getattr(self, 'unstick_events', 0)+1
                    self.stuck_history = []
                    flow = getattr(self, 'navigation_flow', None)
                    if flow is not None:
                        flow.record_navigation_result(6, 'no_progress_10s', now)
                    self.get_logger().warning('No progress for 10 s while driving; backing out')
                    self.cancel_action('no_progress_10s')
                    self.recovery_pending = True
                    self.last_recovery = -100.
        elif getattr(self, 'kind', '') != 'nav':
            self.stuck_history = []
        # Create 3 owns the undock motion and its hazard checks. Its action does
        # not require camera coverage, SLAM or Nav2 readiness to remain active.
        if getattr(self,'kind','')=='undock' and (self.goal_handle or self.pending):
            if not self.sensor_fresh(self.scan_stamp):self.cancel_action('scan_stale_during_undock')
            elif self.now()-self.action_start>65:self.cancel_action('undock_timeout')
            return
        if getattr(self,'pending_camera',None) is not None:self.observe_camera()
        if not self.ready():
            if self.goal_handle or self.pending:self.cancel_action(self.last_readiness_reason)
            self.zero(); return
        flow=getattr(self,'navigation_flow',None)
        if getattr(self,'near_pass',None) is not None:self.near_pass.poll()
        guard=getattr(self,'contact_inspection',None)
        now=self.now()
        if getattr(self, 'kind', '') == 'startup_spin' and (self.goal_handle or self.pending):
            if now-self.action_start > 35. and not self.cancel_reason:
                self.cancel_action('action_timeout')
            return
        if getattr(self, 'start_sweep_remaining', 0.) > 0.:
            if self.contact_pending or self.recovery_pending or (guard and (guard.pressed or guard.phase)):
                self.start_sweep_remaining = 0.
                self.get_logger().warning('Startup sweep yielded to contact recovery')
            elif not (self.goal_handle or self.pending):
                goal = Spin.Goal()
                goal.target_yaw = self.start_sweep_remaining
                goal.time_allowance.sec = 32
                self.start_sweep_attempts += 1
                self.start_sweep_remaining = 0.
                self.scan_after = False
                self.initial_scan = False
                self.send(self.spin, goal, 'startup_spin')
                self.get_logger().info(f'Startup sweep requested {goal.target_yaw:.3f} rad')
                return
        pending_ids = getattr(self, 'registered_pending', set())
        if pending_ids and not self.contact_pending and not self.recovery_pending and not (guard and guard.phase):
            unresolved = {i for i in pending_ids if not self.approach.verified(i)}
            busy = self.goal_handle or self.pending
            current = getattr(self, 'batch_approach_id', None)
            # Continue an actual validated approach. Ordinary inspection and focus may yield.
            if not busy or (self.kind in ('nav','spin') and not self.cancel_reason):
                pending_ids.clear()
                if unresolved and (current not in self.targets or self.approach.verified(current)):
                    if self.near_pass.task and self.near_pass.task.get('source') == 'edge_print':
                        self.near_pass.abort('registered_marker')
                    self.readable_cue = None
                    self.scan_after = False
                    self.resume_action = None
                    self.plan_after = now
                    self.get_logger().info(f'Reassessing exploration for registered IDs {sorted(unresolved)}')
                    if busy:
                        self.cancel_action('registered_marker')
                        return
        if getattr(self, 'readable_cue', None) and self.follow_readable_cue(now):
            return
        focus = getattr(self, 'marker_focus', None)
        if focus and Mission.interrupt_for_hint(self, focus.hints, deferred=True):
            return
        if getattr(self, 'near_pass', None) is not None and self.near_pass.request_print():
            return
        bearing_fresh=guard is not None and guard.bearing is not None and now-guard.bearing[0]<=2.
        protected=any(getattr(getattr(self,name,None),'pending',None) is not None for name in ('marker_look','face_look'))
        active_motion = bool(self.goal_handle or self.pending)
        lap = getattr(getattr(getattr(self,'hybrid',None),'schedule',None),'wall_lap',None)
        can_pause_for_survey = ((lap is None or lap.phase != 'perimeter')
                               and (not active_motion or flow is None or flow.allow_mid_route_wall_inspection))
        periodic_due = bool(guard is not None and can_pause_for_survey and guard.due(now))
        wall=getattr(self,'wall_look',None)
        wall_due=(wall is not None and can_pause_for_survey and self.hybrid is not None and self.hybrid.schedule.phase=="wall" and not protected and not bearing_fresh and not guard.phase
                  and not getattr(getattr(self,'near_pass',None),'protected_navigation',False)
                  and now>=self.spin_blocked_until
                  and (not (self.goal_handle or self.pending) or self.kind=='nav')
                  and wall.check(self,now))
        if self.goal_handle or self.pending:
            hybrid=getattr(self,'hybrid',None)
            if (hybrid is not None and self.kind=='nav' and not self.cancel_reason
                    and (not protected or hybrid.has_fresh_print(self))
                    and not self.contact_pending and not self.recovery_pending and not guard.phase
                    and not bearing_fresh and now>=self.spin_blocked_until
                    and not getattr(hybrid,'active_print_priority',False)
                    and hybrid.should_interrupt(self)):
                self.cancel_action('marker_hint');self.zero();return
            if (guard is not None and not guard.phase and self.kind=='nav' and not protected
                    and not getattr(getattr(self,'near_pass',None),'protected_navigation',False)
                    and not self.cancel_reason and now>=self.spin_blocked_until
                    and (periodic_due or bearing_fresh or wall_due)):
                self.cancel_action('inspection_due');return
            if self.now()-self.action_start>65:self.cancel_action('action_timeout')
            elif (flow is not None and self.kind=='nav' and not self.cancel_reason and flow.stalled(self.now())):
                flow.stalls_cancelled+=1;self.cancel_action('navigation_stalled');self.zero()
            elif (getattr(self,'near_pass',None) is not None and
                  getattr(self.near_pass,'inspectable_route',False) and
                  (flow is None or flow.allow_mid_route_wall_inspection)):self.near_pass.request()
            return
        if guard is not None and guard.phase:
            self.recovery_pending=False
            self.resume_action=None
            step=guard.recovery(now,self.rear_clear,self.backup.server_is_ready(),self.spin.server_is_ready())
            if step=='hold':
                self.zero();return
            guard.action=step
            if step=='backup':
                goal=BackUp.Goal();goal.target.x=.12;goal.speed=.06;goal.time_allowance.sec=5
                self.send(self.backup,goal,step)
            else:
                goal=Spin.Goal();goal.target_yaw=guard.side*math.radians(65);goal.time_allowance.sec=10
                self.send(self.spin,goal,step)
            self.get_logger().info(f'Contact recovery: {step}')
            return
        if guard is not None and not protected and now>=self.spin_blocked_until and (periodic_due or bearing_fresh or wall_due):
            self.scan_after=True
        if getattr(self,'near_pass',None) is not None and self.near_pass.tick():return
        if getattr(self,'resume_action',None) is not None:
            kind,goal=self.resume_action;self.resume_action=None
            if kind=='spin' and abs(goal.target_yaw)<1e-6:
                if flow is None:self.plan_after=self.now()+1.5
                else:flow.begin_settle(self.now(),self.camera_diagnostics['accepted']);self.plan_after=self.now()
                self.zero();return
            if kind=='nav':goal.pose.header.stamp=self.get_clock().now().to_msg()
            self.get_logger().info(f'Resuming {kind} after sensor recovery')
            self.send(self.nav if kind=='nav' else self.spin,goal,kind);return
        if self.now()<self.plan_after: self.zero(); return
        if flow is not None and flow.waiting_for_frames(self.now(),self.camera_diagnostics['accepted']):self.zero();return
        if self.recovery_pending:
            self.recovery_pending=False
            if self.rear_clear and self.backup.server_is_ready() and self.now()-self.last_recovery>=15. and (flow is None or flow.backup_allowed(self.now())):
                self.last_recovery=self.now(); goal=BackUp.Goal(); goal.target.x=.20; goal.speed=.08; goal.time_allowance.sec=8
                self.send(self.backup,goal,'backup'); return
        if self.scan_after and self.now()<self.spin_blocked_until: self.scan_after=False
        if self.scan_after:
            self.scan_after=False; goal=Spin.Goal()
            protected_look=any(getattr(getattr(self,name,None),'pending',None) is not None for name in ('marker_look','face_look'))
            goal.target_yaw=self.observation_turn()
            if guard is not None:
                if not protected_look:
                    if bearing_fresh:
                        goal.target_yaw=max(-.8,min(.8,guard.bearing[1]))
                        guard.bearing=None
                        if abs(goal.target_yaw)<.08:
                            goal.target_yaw=0.;protected_look=True
                    elif wall_due:
                        wall_angle=wall.consume(self,now)
                        if wall_angle is not None:
                            goal.target_yaw=wall_angle
                    elif abs(goal.target_yaw)<1e-6 and (self.initial_scan or guard.due(now)):
                        goal.target_yaw=guard.survey_angle()
                if goal.target_yaw or bearing_fresh or wall_due or self.initial_scan:
                    guard.looked(now)
            self.initial_scan=False; goal.time_allowance.sec=15
            self.get_logger().info(f'Bounded observation turn: {math.degrees(goal.target_yaw):.0f} deg')
            if abs(goal.target_yaw)<1e-6:
                if flow is None:self.plan_after=self.now()+1.5;self.zero();return
                if protected_look:
                    flow.begin_settle(self.now(),self.camera_diagnostics['accepted']);self.zero();return
                flow.skipped_zero_turns+=1
            else:self.send(self.spin,goal,'spin');return
        self.marker_look.clear('replan')
        if getattr(self,'face_look',None) is not None:self.face_look.clear('replan')
        tr=self.tf.lookup_transform(self.p('map_frame'),self.p('base_frame'),Time())
        m=self.map; data=np.array(m.data).reshape(m.info.height,m.info.width)
        dock=self.known_dock_pose(self.p('map_frame'))
        if dock is not None:
            dock_geometry.mark(data,m.info.resolution,(m.info.origin.position.x,m.info.origin.position.y),dock)
        if self.p('policy_file'):
            try:
                path=Path(self.p('policy_file')).expanduser(); changed=path.stat().st_mtime
                if changed!=self.policy_mtime:
                    weights,settings=self.load_configuration(path)
                    self.weights,self.surface_settings=weights,settings; self.policy_mtime=changed
                    self.get_logger().info(f'Loaded policy checkpoint: {path}')
            except (OSError,ValueError,KeyError) as exc: self.get_logger().warning(str(exc))
        viewed=np.zeros(data.shape,dtype=bool)
        if self.surface_mode:
            viewed=self.coverage.grid(data,m.info.resolution,(m.info.origin.position.x,m.info.origin.position.y))
        else:
            for x,y in self.view_samples:
                col=int((x-m.info.origin.position.x)/m.info.resolution); row=int((y-m.info.origin.position.y)/m.info.resolution)
                if 0<=row<data.shape[0] and 0<=col<data.shape[1]: viewed[row,col]=True
        q=tr.transform.rotation; heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        option_settings={'preferred_clearance':self.surface_settings['view_standoff']} if self.surface_settings else {}
        if self.edge_mode:
            calibration=getattr(self,'inspection_camera',None)
            option_settings['inspection_focal']=calibration[0] if calibration is not None else 0.
            option_settings['start_ignore_radius']=getattr(self,'planner_start_ignore_radius_m',0.)
        if self.edge_mode and getattr(self,'wall_look',None) is not None:
            option_settings['return_paths']=True
        choices,previous=self.policy_options(data,m.info.resolution,(m.info.origin.position.x,m.info.origin.position.y),
                           (tr.transform.translation.x,tr.transform.translation.y),heading,self.excluded,viewed,**option_settings)
        memory_transform=None
        if self.edge_mode:
            try:
                mt=self.tf.lookup_transform('odom',self.p('map_frame'),Time()).transform
                mq=mt.rotation
                memory_transform=(mt.translation.x,mt.translation.y,math.atan2(2*(mq.w*mq.z+mq.x*mq.y),1-2*(mq.y*mq.y+mq.z*mq.z)))
                choices=self.exploration_memory.filter(choices,memory_transform,self.now())
                if getattr(self,'wall_look',None) is not None:
                    penalize_routes(choices,previous,data.shape,m.info.resolution,
                        (m.info.origin.position.x,m.info.origin.position.y),memory_transform,self.exploration_memory.cells)
            except Exception as exc:self.get_logger().debug(f'Exploration memory TF unavailable: {exc}')
        if flow is not None:choices=flow.filter_goals(choices,memory_transform,self.now())
        selected=(choose_before_deadline(choices,self.weights,self.p('duration')-(self.now()-self.started))
                  if self.surface_settings.get('deadline_aware') else self.policy_choose(choices,self.weights))
        if self.edge_mode:
            focus=self.marker_focus.choose(data,m.info.resolution,(m.info.origin.position.x,m.info.origin.position.y),
                (tr.transform.translation.x,tr.transform.translation.y),heading,choices,self.targets,self.get_clock().now().nanoseconds/1e9)
            if focus is not None:
                selected=focus;choices.append(focus)
                self.get_logger().info(f"Following up accepted marker {focus['focus_id']}; seeking a readable close view")
                audit=Path(self.p('output_dir'))/'marker_followups.jsonl'
                try:
                    with audit.open('a') as stream:
                        stream.write(json.dumps(dict(stamp=self.now(),id=focus['focus_id'],
                            point=self.marker_focus.hints[focus['focus_id']]['point'].tolist(),
                            observer=self.marker_focus.hints[focus['focus_id']]['observer'].tolist(),
                            goal=[focus['x'],focus['y'],focus['yaw']],stationary=bool(focus.get('stationary')),
                            attempt=self.marker_focus.attempts.get(focus['focus_id'],0)+1,
                            observed_min_edge_pixels=focus.get('observed_min_edge_pixels'),
                            predicted_min_edge_pixels=focus.get('predicted_min_edge_pixels'),
                            view_resolution_penalty_s=focus.get('view_resolution_penalty_s')))+'\n')
                except OSError as exc:self.get_logger().warning(f'Could not save marker follow-up audit: {exc}')
        approach=self.approach.choose(data,m.info.resolution,(m.info.origin.position.x,m.info.origin.position.y),
            (tr.transform.translation.x,tr.transform.translation.y),heading,choices,self.targets,self.get_clock().now().nanoseconds/1e9)
        if approach is not None:
            selected=approach;choices.append(approach)
            self.get_logger().info(f"Approaching confirmed ID {approach['approach_id']} for close RGB verification")
        if flow is not None and selected is not None and flow.blocked(selected,memory_transform,self.now()):
            choices=flow.filter_goals(choices,memory_transform,self.now());selected=self.policy_choose(choices,self.weights)
        hybrid=getattr(self,'hybrid',None)
        if hybrid is not None:
            try:
                combined=hybrid.select(self,selected,data,viewed,(tr.transform.translation.x,tr.transform.translation.y,heading),memory_transform,choices=choices)
                if combined is None and hybrid.band_rejected_selected:
                    selected = None
                if combined is not None and combined is not selected:
                    choices.append(combined)
                    selected=combined
            except Exception as exc:
                self.get_logger().warning(f'Hybrid proposal unavailable, using Flow choice: {exc}')
        try:
            save_policy_audit(self.p('output_dir'), self.policy_decisions, self.now()-self.started,
                self.p('planner'), data, m.info.resolution,
                (m.info.origin.position.x, m.info.origin.position.y),
                (tr.transform.translation.x, tr.transform.translation.y), heading,
                self.excluded, viewed, self.weights, choices, selected)
            self.policy_decisions+=1
        except Exception as exc:
            self.get_logger().warning(f'Could not save policy input audit: {exc}')
        if selected is None:
            # Retry old frontiers after scanning rather than finishing before the deadline.
            self.excluded=self.excluded[-5:]
            self.empty_plans=getattr(self,'empty_plans',0)+1
            flow = getattr(self, 'navigation_flow', None)
            backup_possible = (self.rear_clear and self.backup.server_is_ready()
                               and self.now()-self.last_recovery >= 15.
                               and (flow is None or flow.backup_allowed(self.now())))
            if self.empty_plans>=3 and not backup_possible and self.open_escape():
                # .59: no reachable goal and no room to back up. Let Nav2 drive a short
                # step toward the most open scan direction instead of waiting.
                # .67: also when a backup would not run (cooldown, flow block, no server);
                # on 2026-11-27 the robot repeated "checking safe backup" for 50 s.
                self.empty_plans=0
                return
            if self.empty_plans>=3:
                # A scan cannot reconnect an inflated start indefinitely. Try
                # the existing scan-checked backup recovery instead of spinning.
                self.empty_plans=0;self.recovery_pending=True;self.scan_after=False
                self.spin_blocked_until=self.now()+15.;self.plan_after=self.now()+1.
                self.get_logger().warning('No reachable observation goals after three scans; checking safe backup')
                self.zero();return
            if self.now()>=self.spin_blocked_until:
                self.scan_after=True
            else:
                self.plan_after=self.now()+2.; self.zero()
            return
        self.empty_plans=0
        marker_task_dispatched(self.marker_focus, self.approach, selected, self.get_clock().now().nanoseconds/1e9)
        if hybrid is not None:
            hybrid.dispatched(self,selected)
        if memory_transform is not None:
            self.exploration_memory.selected(selected,memory_transform)
            try:
                with (Path(self.p('output_dir'))/'exploration_memory.jsonl').open('a') as stream:
                    stream.write(json.dumps(dict(elapsed_seconds=self.now()-self.started,
                        selected=[selected['x'],selected['y'],selected['yaw']],
                        inspection_focal_px=option_settings.get('inspection_focal'),
                        readable_distance_m=edge_policy.readable_distance(option_settings.get('inspection_focal')),
                        marker_override='approach_id' in selected or 'focus_id' in selected,
                        **self.exploration_memory.report()))+'\n')
            except OSError as exc:self.get_logger().warning(str(exc))
        if getattr(self,'near_pass',None) is not None:
            # Preserve confirmed markers and print revisits. A box/face
            # transit may pause for another visible side and resume.
            self.near_pass.protected_navigation=any(k in selected for k in ('approach_id','focus_id','print_point','box_recovery_walk_m'))
            self.near_pass.inspectable_route=False  # The phase scheduler owns surface observations.
        if flow is not None:flow.selected(selected,memory_transform)
        self.marker_look.select(selected,memory_transform)
        self.face_look.select(selected,memory_transform)
        self.batch_approach_id = selected.get('approach_id')
        self.next_scan_angle=selected.get('scan_angle',2*math.pi/3)
        if selected.get('stationary'):
            self.scan_after=True;self.zero();return
        x,y,a=selected['x'],selected['y'],selected['yaw']; self.destination=(x,y)
        self.get_logger().info(f'Navigation goal ({x:.2f}, {y:.2f}), {len(choices)} feasible candidates')
        g=NavigateToPose.Goal(); g.pose.header.frame_id=self.p('map_frame'); g.pose.header.stamp=self.get_clock().now().to_msg()
        g.pose.pose.position.x=x; g.pose.pose.position.y=y; g.pose.pose.orientation.z=math.sin(a/2); g.pose.pose.orientation.w=math.cos(a/2)
        self.send(self.nav,g,'nav')


def main(args=None):
    rclpy.init(args=args); n=Mission()
    try: rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        if rclpy.ok(): n.zero()
        if n.save_proc and n.save_proc.poll() is None: n.save_proc.terminate()
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
