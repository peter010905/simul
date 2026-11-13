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
from osp_mission.marker_directed_look import MarkerDirectedLook
from osp_mission.face_directed_look import FaceDirectedLook
from osp_mission.exploration_memory import ExplorationMemory
from osp_mission.near_pass_runtime import NearPassRuntime


class Mission(Node):
    def __init__(self):
        super().__init__('mission')
        for k,v in {'duration':600.,'output_dir':str(Path.home()/'simul/results'),
                    'map_frame':'map','base_frame':'base_link','odom_frame':'odom','scan_topic':'scan',
                    'undock':False,'policy_file':'','planner':'legacy','require_drive_controller':False}.items(): self.declare_parameter(k,v)
        self.p=lambda k:self.get_parameter(k).value
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
        self.inspect=self.p('planner')=='inspection'
        self.directional=self.p('planner')=='directional'
        self.edge_mode=self.p('planner')=='edge'
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
        if self.edge_mode:
            self.create_subscription(String,'mission/marker_observations',self.on_marker_hint,10)
            self.create_subscription(String,'mission/marker_candidates',self.on_marker_hint,10)
        self.latest_scan=None;self.last_camera=0.;self.last_camera_stamp=-1
        self.scan_stamp=None;self.camera_stamp=None;self.pending_camera=None
        self.cancel_reason='';self.undock_diagnostics={};self.last_readiness_reason=''
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
        self.approach=MarkerApproach()
        self.graph_observations=[]
        self.create_subscription(String,'mission/marker_observations',self.on_approach_image,10)
        self.map=None; self.targets={}; self.last_scan=0.; self.state='idle'
        self.goal_handle=None; self.pending=False; self.excluded=[]; self.action_start=0.; self.scan_after=True
        self.started=0.; self.save_proc=None; self.serialize_future=None; self.save_deadline=0.
        self.policy_decisions=0
        self.create_timer(.5,self.tick)

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
    def on_hazard(self,m):
        bump=any(d.type==1 for d in m.detections)
        if bump and self.state=='running' and self.now()-self.last_bump>2.:
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
        self.targets={a.id:(a.pose.position.x,a.pose.position.y) for a in m.markers if a.ns=='point'}
        if getattr(self,'approach',None) is not None:self.approach.reconcile_targets(self.targets)
    def on_marker_hint(self,m):
        try:
            evidence=json.loads(m.data)
            observer=evidence.get('camera_origin_map')
            if observer is None:
                tr=self.tf.lookup_transform(self.p('map_frame'),self.p('base_frame'),Time(seconds=float(evidence['stamp'])))
                observer=(tr.transform.translation.x,tr.transform.translation.y)
            accepted=self.marker_focus.observe(evidence,observer[:2],self.get_clock().now().nanoseconds/1e9,self.p('map_frame'))
            if (accepted and self.state=='running' and getattr(self,'kind','')=='nav'
                    and (self.goal_handle or self.pending) and not self.cancel_reason
                    and self.marker_focus.request_interrupt(int(evidence['id']),self.targets,self.get_clock().now().nanoseconds/1e9)):
                self.get_logger().info(f"Pausing navigation to verify accepted marker {evidence['id']}")
                self.cancel_action('marker_hint');self.zero()
        except (KeyError,TypeError,ValueError):
            return
        except Exception as exc:
            self.get_logger().debug(f'Marker follow-up unavailable: {exc}')
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
    def publish_known_obstacles(self):
        if self.known_dock_pose(self.p('map_frame')) is None:return
        header=Header(frame_id='std_dock_link',stamp=self.get_clock().now().to_msg())
        self.known_obstacles.publish(point_cloud2.create_cloud_xyz32(header,dock_geometry.points()))
    def send(self,client,goal,kind):
        self.pending=True; self.action_start=self.now(); self.kind=kind;self.cancel_reason=''
        self.active_goal=copy.deepcopy(goal);self.spin_progress=0.;self.action_generation+=1
        generation=self.action_generation
        feedback={'feedback_callback':lambda m:self.spin_feedback(m,generation)} if kind=='spin' else {}
        client.send_goal_async(goal,**feedback).add_done_callback(self.accepted)
    def spin_feedback(self,message,generation):
        if generation!=self.action_generation or self.kind!='spin':return
        distance=float(message.feedback.angular_distance_traveled)
        if math.isfinite(distance):self.spin_progress=distance
    def cancel_action(self,reason):
        if self.cancel_reason:return
        self.cancel_reason=reason
        self.get_logger().warning(f'Cancel {self.kind}: {reason}')
        if self.goal_handle:self.goal_handle.cancel_goal_async()
    def accepted(self,future):
        self.pending=False
        try: handle=future.result()
        except Exception as exc:
            self.contact_pending=False
            self.get_logger().error(str(exc))
            if getattr(self,'marker_look',None) is not None:self.marker_look.clear('action_request_error')
            if getattr(self,'face_look',None) is not None:self.face_look.clear('action_request_error')
            if self.kind=='undock':self.finish('undock_request_failed')
            return
        if not handle.accepted:
            self.contact_pending=False
            if getattr(self,'marker_look',None) is not None:self.marker_look.clear('action_rejected')
            if getattr(self,'face_look',None) is not None:self.face_look.clear('action_rejected')
            if getattr(self,'near_pass',None) is not None and self.near_pass.rejected(self.kind):return
            if self.kind=='nav': self.excluded.append(self.destination)
            if self.kind=='spin':
                self.spin_blocked_until=self.now()+30.; self.scan_after=False; self.recovery_pending=True
            if self.kind=='undock': self.finish('undock_rejected')
            return
        self.goal_handle=handle
        if self.state!='running' or self.cancel_reason: handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(self.completed)
    def completed(self,future):
        self.goal_handle=None
        try: result=future.result()
        except Exception as exc:
            if getattr(self,'marker_look',None) is not None:self.marker_look.clear('action_result_error')
            if getattr(self,'face_look',None) is not None:self.face_look.clear('action_result_error')
            if self.kind=='undock':
                self.undock_diagnostics={'error':str(exc),'cancel_reason':self.cancel_reason}
                self.finish('undock_result_error')
            return
        if self.state!='running': return
        if getattr(self,'contact_pending',False):
            if getattr(self,'marker_look',None) is not None:self.marker_look.clear('bump_contact')
            if getattr(self,'face_look',None) is not None:self.face_look.clear('bump_contact')
            self.contact_pending=False;self.resume_action=None;self.scan_after=False
            self.recovery_pending=self.kind!='backup';self.plan_after=self.now()+.5
            if self.kind=='nav':self.excluded.append(self.destination)
            return
        if getattr(self,'near_pass',None) is not None and self.near_pass.completed(result.status,self.kind,getattr(self,'cancel_reason','')):return
        sensor_reasons={'processed_camera_stale','scan_stale','map_unavailable','map_to_base_tf_unavailable','nav2_unavailable','drive_unavailable'}
        if result.status==5 and getattr(self,'cancel_reason','') in sensor_reasons and self.kind in ('nav','spin'):
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
            if getattr(self,'exploration_memory',None) is not None:self.exploration_memory.completed(result.status==4,self.now())
            self.excluded.append(self.destination); self.scan_after=result.status==4
            if result.status!=4 and getattr(self,'marker_look',None) is not None:self.marker_look.clear('navigation_failed')
            if result.status!=4 and getattr(self,'face_look',None) is not None:self.face_look.clear('navigation_failed')
        elif self.kind=='spin' and result.status!=4:
            self.spin_blocked_until=self.now()+30.; self.scan_after=False; self.recovery_pending=True
            self.get_logger().warning('Spin blocked; suppressing retries for 30 s and checking a safe retreat')
        elif self.kind=='spin':
            self.plan_after=self.now()+1.5
        elif self.kind=='backup':
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
                'approach':self.approach.report(),'reason':self.reason,'elapsed_seconds':self.elapsed,'targets':len(self.targets),'collisions':self.collision_count,
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
        self.status.publish(String(data=json.dumps({'state':self.state,'approached_ids':self.approach.report()['verified_ids'] if getattr(self,'approach',None) is not None else [],'targets':len(self.targets),'collisions':self.collision_count,
            'elapsed_seconds':max(0.,self.now()-self.started) if self.started else 0.,
            'active_action':getattr(self,'kind','') if self.goal_handle or self.pending else '',
            'readiness_reason':getattr(self,'last_readiness_reason',''),
            'sensor_interruptions':getattr(self,'sensor_interruptions',0),
            'nav2_lifecycle':self.nav2_state.states if getattr(self,'nav2_state',None) is not None else {},
            'drive_controllers':self.drive_state.states if getattr(self,'drive_state',None) is not None else {},
            'confirmed_surface_cells':int(np.count_nonzero(self.coverage.viewed)) if getattr(self,'coverage',None) is not None and self.coverage.viewed is not None else 0,
            'confirmed_face_anchors':len(getattr(getattr(self,'coverage',None),'anchors',())),
            'reassociated_face_records':int(getattr(getattr(self,'coverage',None),'reassociated_faces',0)),
            'camera_diagnostics':getattr(self,'camera_diagnostics',{})})))
        if self.state=='saving': self.save_tick(); return
        if self.state=='done': self.zero(); return
        if self.state!='running': return
        try:
            visit=self.tf.lookup_transform('odom',self.p('base_frame'),Time()).transform.translation
            self.exploration_memory.observe(visit.x,visit.y,self.now())
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
        if getattr(self,'near_pass',None) is not None:self.near_pass.poll()
        if self.goal_handle or self.pending:
            if self.now()-self.action_start>65:self.cancel_action('action_timeout')
            elif getattr(self,'near_pass',None) is not None:self.near_pass.request()
            return
        if getattr(self,'near_pass',None) is not None and self.near_pass.tick():return
        if getattr(self,'resume_action',None) is not None:
            kind,goal=self.resume_action;self.resume_action=None
            if kind=='spin' and abs(goal.target_yaw)<1e-6:
                self.plan_after=self.now()+1.5;self.zero();return
            if kind=='nav':goal.pose.header.stamp=self.get_clock().now().to_msg()
            self.get_logger().info(f'Resuming {kind} after sensor recovery')
            self.send(self.nav if kind=='nav' else self.spin,goal,kind);return
        if self.now()<self.plan_after: self.zero(); return
        if self.recovery_pending:
            self.recovery_pending=False
            if self.rear_clear and self.backup.server_is_ready() and self.now()-self.last_recovery>=15.:
                self.last_recovery=self.now(); goal=BackUp.Goal(); goal.target.x=.20; goal.speed=.08; goal.time_allowance.sec=8
                self.send(self.backup,goal,'backup'); return
        if self.scan_after and self.now()<self.spin_blocked_until: self.scan_after=False
        if self.scan_after:
            self.scan_after=False; goal=Spin.Goal()
            goal.target_yaw=self.observation_turn()
            self.initial_scan=False; goal.time_allowance.sec=15
            self.get_logger().info(f'Bounded observation turn: {math.degrees(goal.target_yaw):.0f} deg')
            if abs(goal.target_yaw)<1e-6:
                self.plan_after=self.now()+1.5; self.zero(); return
            self.send(self.spin,goal,'spin'); return
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
        choices,_=self.policy_options(data,m.info.resolution,(m.info.origin.position.x,m.info.origin.position.y),
                           (tr.transform.translation.x,tr.transform.translation.y),heading,self.excluded,viewed,**option_settings)
        memory_transform=None
        if self.edge_mode:
            try:
                mt=self.tf.lookup_transform('odom',self.p('map_frame'),Time()).transform
                mq=mt.rotation
                memory_transform=(mt.translation.x,mt.translation.y,math.atan2(2*(mq.w*mq.z+mq.x*mq.y),1-2*(mq.y*mq.y+mq.z*mq.z)))
                choices=self.exploration_memory.filter(choices,memory_transform,self.now())
            except Exception as exc:self.get_logger().debug(f'Exploration memory TF unavailable: {exc}')
        selected=(choose_before_deadline(choices,self.weights,self.p('duration')-(self.now()-self.started))
                  if self.surface_settings.get('deadline_aware') else self.policy_choose(choices,self.weights))
        if self.edge_mode:
            focus=self.marker_focus.choose(data,m.info.resolution,(m.info.origin.position.x,m.info.origin.position.y),
                (tr.transform.translation.x,tr.transform.translation.y),heading,choices,self.targets,self.get_clock().now().nanoseconds/1e9)
            if focus is not None:
                selected=focus;choices.append(focus)
                self.get_logger().info(f"Following up accepted marker {focus['focus_id']}; three-frame confirmation still required")
                audit=Path(self.p('output_dir'))/'marker_followups.jsonl'
                try:
                    with audit.open('a') as stream:
                        stream.write(json.dumps(dict(stamp=self.now(),id=focus['focus_id'],
                            point=self.marker_focus.hints[focus['focus_id']]['point'].tolist(),
                            observer=self.marker_focus.hints[focus['focus_id']]['observer'].tolist(),
                            goal=[focus['x'],focus['y'],focus['yaw']],stationary=bool(focus.get('stationary')),
                            attempt=self.marker_focus.attempts[focus['focus_id']],
                            observed_min_edge_pixels=focus.get('observed_min_edge_pixels'),
                            predicted_min_edge_pixels=focus.get('predicted_min_edge_pixels'),
                            view_resolution_penalty_s=focus.get('view_resolution_penalty_s')))+'\n')
                except OSError as exc:self.get_logger().warning(f'Could not save marker follow-up audit: {exc}')
        approach=self.approach.choose(data,m.info.resolution,(m.info.origin.position.x,m.info.origin.position.y),
            (tr.transform.translation.x,tr.transform.translation.y),heading,choices,self.targets,self.get_clock().now().nanoseconds/1e9)
        if approach is not None:
            selected=approach;choices.append(approach)
            self.get_logger().info(f"Approaching confirmed ID {approach['approach_id']} for close RGB verification")
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
        if getattr(self,'near_pass',None) is not None:self.near_pass.protected_navigation='approach_id' in selected or 'focus_id' in selected
        self.marker_look.select(selected,memory_transform)
        self.face_look.select(selected,memory_transform)
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
