"""ArUco observations anchored in the map at the camera exposure time."""
import math
import copy
import json
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile,ReliabilityPolicy,qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import message_filters
from osp_mission.hybrid_geometry import preview_geometry
from osp_mission.marker_relook import VisualRange, sight_in_odom
from osp_mission.marker_pair import SeparateMarkers, decode
from osp_mission.landmark_identity import resolved_ids
from osp_mission.provisional_marker_cues import ProvisionalMarkerCues
from osp_mission.landmark_reacquisition import ObservationConsensus


class Detector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        defaults={'rgb_topic':'oakd/rgb/preview/image_raw','depth_topic':'oakd/rgb/preview/depth',
                  'camera_info_topic':'oakd/rgb/preview/camera_info','scan_topic':'scan',
                  'map_frame':'map','odom_frame':'odom','confirmations':3,'min_marker_pixels':18}
        for k,v in defaults.items(): self.declare_parameter(k,v)
        self.p=lambda k:self.get_parameter(k).value
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
        self.bridge=CvBridge(); self.info=None; self.scan=None; self.depth=None
        self.provisional=ProvisionalMarkerCues()
        from osp_mission.contact_inspection import BearingCue
        self.bearing_cue=BearingCue()
        self.readable_bearings = BearingCue()
        from osp_mission.camera_multiview import CameraVisualRange
        from osp_mission.camera_work import ExposureQueue, PipelineTiming
        self.visual_range=CameraVisualRange()
        self.exposure_queue=ExposureQueue()
        self.pipeline_timing=PipelineTiming()
        from osp_mission.sensor_history import SensorHistory
        self.sensor_history=SensorHistory()
        self.bearing_pub=self.create_publisher(String,'mission/marker_bearing',10)
        from osp_mission.hybrid_passing import PassingGlance
        self.print_glance=PassingGlance()
        self.glance_pub=self.create_publisher(String,'mission/anonymous_glance',10)
        self.small_wall_pub=self.create_publisher(String,'mission/small_wall_bearings',10)
        self.print_pub=self.create_publisher(String,'mission/anonymous_print',10)
        self.candidate_pub=self.create_publisher(String,'mission/marker_candidates',10)
        self.confirmation_guard=ObservationConsensus()
        self.observations={}; self.pub=self.create_publisher(MarkerArray,'targets',10)
        self.diagnostics={'frames':0,'decoded':{},'small':0,'border':0,'no_range':0,'horizontal':0,'inconsistent':0,'accepted':0,'errors':0,'last_error':''}
        self.diagnostic_pub=self.create_publisher(String,'mission/detector_status',10)
        self.id_diagnostics={}
        self.observation_pub=self.create_publisher(String,'mission/marker_observations',10)
        # A heartbeat represents an actually processed frame, including frames
        # without a marker. LiDAR alone must not mark a surface camera-inspected.
        self.processed_camera=self.create_publisher(CameraInfo,'mission/processed_camera',qos_profile_sensor_data)
        latest_image=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CameraInfo,self.p('camera_info_topic'),self.on_info,latest_image)
        self.create_subscription(LaserScan,self.p('scan_topic'),self.on_scan,qos_profile_sensor_data)
        self.create_subscription(Image,self.p('depth_topic'),self.on_depth,latest_image)
        self.create_subscription(Image,self.p('rgb_topic'),self.enqueue_rgb,latest_image)
        self.create_timer(.02,self.drain_rgb)
        self.separate_markers=SeparateMarkers()
        self.dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.create_timer(.5,self.publish)

    def on_info(self,m): self.info=m
    def reject(self, ident, reason):
        self.diagnostics[reason]+=1
        detail=self.id_diagnostics.setdefault(str(int(ident)),{})
        detail[reason]=detail.get(reason,0)+1
    def on_scan(self,m):
        self.scan=m;self.sensor_history.scan(m)
    def on_depth(self,m):
        self.depth=m;self.sensor_history.depth(m)

    def lidar_point(self, ray, frame, stamp):
        scan=self.scan
        if scan is None or abs((Time.from_msg(scan.header.stamp)-stamp).nanoseconds)>250_000_000:
            return None
        tr=self.tf.lookup_transform_full(scan.header.frame_id,Time.from_msg(scan.header.stamp),
            frame,stamp,self.p('odom_frame'),timeout=Duration(seconds=0.))
        origin=PointStamped(); origin.header.frame_id=frame
        direction=PointStamped(); direction.point.x,direction.point.y,direction.point.z=map(float,ray)
        o=do_transform_point(origin,tr).point; d=do_transform_point(direction,tr).point
        o=np.array([o.x,o.y]); d=np.array([d.x,d.y])-o
        if np.linalg.norm(d)<.1: return None
        angle=math.atan2(d[1],d[0]); points=[]
        for i,r in enumerate(scan.ranges):
            a=scan.angle_min+i*scan.angle_increment
            delta=math.atan2(math.sin(a-angle),math.cos(a-angle))
            if abs(delta)<.07 and math.isfinite(r) and scan.range_min<r<scan.range_max:
                points.append([r*math.cos(a),r*math.sin(a)])
        if len(points)<4: return None
        points=np.array(points); center=np.median(points,axis=0)
        _,_,v=np.linalg.svd(points-center); normal=v[-1]
        if np.max(np.abs((points-center)@normal))>.06: return None
        denom=normal@d
        if abs(denom)<.05: return None
        distance=float(normal@(center-o)/denom)
        return ray*distance if .15<distance<8 else None

    def visual_sighting(self, ident, ray, pixels, frame, stamp):
        # on_rgb already used the calibrated horizontal model once, including
        # LiDAR, triangulation and processed-camera coverage geometry.
        timestamp = stamp.nanoseconds / 1e9
        angle = -math.atan(ray[0])
        cue = self.readable_bearings.observe(ident, timestamp, angle)
        transform = None
        try:
            transform = self.tf.lookup_transform(self.p('odom_frame'), frame, stamp).transform
        except Exception as exc:
            self.get_logger().debug(f'Marker exposure pose unavailable: {exc}')
        if transform is not None:
            origin, direction = sight_in_odom(transform, ray)
        if cue:
            evidence = dict(id=ident, stamp=timestamp, angle=angle, readable=True)
            if transform is not None:
                evidence.update(yaw_odom=math.atan2(direction[1], direction[0]),
                                origin_odom=origin[:2].tolist())
            self.bearing_pub.publish(String(data=json.dumps(evidence)))
        if transform is None:
            return None
        point = self.visual_range.observe(ident, timestamp, origin, direction, pixels)
        if point is None:
            return None
        return ray * float((point-origin) @ direction / np.linalg.norm(ray))

    def enqueue_rgb(self,msg):
        if self.info is None:return
        if (self.info.width,self.info.height)!=(msg.width,msg.height):
            self.diagnostics['calibration_mismatch']=self.diagnostics.get('calibration_mismatch',0)+1
            return
        self.exposure_queue.offer(msg,self.info,Time.from_msg(msg.header.stamp).nanoseconds/1e9,
                                  self.get_clock().now().nanoseconds/1e9)

    def drain_rgb(self):
        now=self.get_clock().now().nanoseconds/1e9
        def ready(row):
            frame=row.message.header.frame_id;stamp=Time.from_msg(row.message.header.stamp)
            return all(self.tf.can_transform(self.p(target),frame,stamp)
                       for target in ('map_frame','odom_frame'))
        row=self.exposure_queue.take(now,ready)
        if row is None:return
        latest_info,latest_scan,latest_depth=self.info,self.scan,self.depth
        self.info=row.calibration
        self.scan,self.depth=self.sensor_history.select(row.message)
        started=time.perf_counter()
        try:self.on_rgb(row.message)
        finally:
            self.info,self.scan,self.depth=latest_info,latest_scan,latest_depth
            self.pipeline_timing.record(row.stamp,self.get_clock().now().nanoseconds/1e9,time.perf_counter()-started)

    def on_rgb(self,msg):
        self.diagnostics['frames']+=1
        if self.info is None: return
        processed=None
        try:
            image=self.bridge.imgmsg_to_cv2(msg,'mono8')
            k=preview_geometry(self.info).k
            if not np.isfinite(k).all() or k[0]<=0 or k[4]<=0 or self.info.width!=msg.width or self.info.height!=msg.height: return
            small=[]
            corners,ids=decode(image,self.dictionary,small)
            if small:
                try:
                    from osp_mission.wall_route import camera_pose
                    stamp=Time.from_msg(msg.header.stamp)
                    tr=self.tf.lookup_transform(self.p('odom_frame'),msg.header.frame_id,stamp)
                    camera=camera_pose(tr.transform)
                    if camera is not None:
                        cal=preview_geometry(self.info)
                        bearings=[(camera[2]-math.atan2(u-cal.k[2],cal.k[0]),size) for u,_,size,_ in small]
                        self.small_wall_pub.publish(String(data=json.dumps(dict(stamp=stamp.nanoseconds/1e9,
                            origin=camera[:2], bearings=bearings))))
                except Exception as exc:
                    self.get_logger().debug(f'Small wall cue projection unavailable: {exc}')
            accepted_quads={}
            processed=copy.deepcopy(self.info);processed.header=msg.header
            processed.k=list(k)
            hardware = not self.p('use_sim_time')
            if ids is None or hardware:
                try:
                    from osp_mission.hybrid import publish_print
                    publish_print(self, msg, image, decoded=corners if ids is not None else (),
                                  min_edge_pixels=10. if hardware else 0.)
                except Exception as exc:
                    self.get_logger().debug(f'Anonymous print projection unavailable: {exc}')
            if ids is None:
                return
            # CameraInfo distortion is applied before back projection.
            for c,ident in zip(corners,ids.flatten()):
                try:
                    key=str(int(ident));self.diagnostics['decoded'][key]=self.diagnostics['decoded'].get(key,0)+1
                    edges=np.linalg.norm(c[0]-np.roll(c[0],1,axis=0),axis=1)
                    detail=self.id_diagnostics.setdefault(key,{})
                    detail.update(last_min_edge_pixels=float(edges.min()),last_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
                    provisional = edges.min()<self.p('min_marker_pixels')
                    if provisional:
                        self.reject(ident,'small')
                        if edges.min()<10.:continue
                    if np.any(c[0][:,0]<4) or np.any(c[0][:,1]<4) or np.any(c[0][:,0]>msg.width-5) or np.any(c[0][:,1]>msg.height-5): self.reject(ident,'border');continue
                    uv=c[0].mean(axis=0)
                    normalized=cv2.undistortPoints(np.array([[uv]],dtype=np.float64),
                        np.array(k).reshape(3,3),np.array(self.info.d))[0,0]
                    ray=np.array([normalized[0],normalized[1],1.])
                    xyz=None; stamp=Time.from_msg(msg.header.stamp)
                    dep=self.depth
                    if dep is not None and abs((Time.from_msg(dep.header.stamp)-stamp).nanoseconds)<150_000_000 and dep.width==msg.width and dep.height==msg.height:
                        # Registered depth only; sim uses the same RGBD sensor.
                        depth=self.bridge.imgmsg_to_cv2(dep,'passthrough')
                        u,v=map(int,uv); patch=depth[max(0,v-2):v+3,max(0,u-2):u+3].astype(float)
                        if dep.encoding=='16UC1': patch*=.001
                        valid=patch[np.isfinite(patch)&(patch>.15)&(patch<8.)]
                        if valid.size>=5: xyz=ray*float(np.median(valid))
                    frame=msg.header.frame_id
                    source = 'depth'
                    visual = None
                    if not provisional:
                        visual = self.visual_sighting(int(ident), ray, float(edges.min()), frame, stamp)
                    if xyz is None:
                        xyz = self.lidar_point(ray,frame,stamp)
                        source = 'lidar'
                        if visual is not None and (xyz is None or
                                (len(self.observations.get(int(ident), ())) < self.p('confirmations')
                                 and np.linalg.norm(visual-xyz) > .35)):
                            xyz = visual
                            source = 'multiview'
                            self.diagnostics['multiview_ranges'] = self.diagnostics.get('multiview_ranges', 0) + 1
                    if xyz is None:
                        self.reject(ident,'no_range')
                        timestamp=stamp.nanoseconds/1e9
                        angle=-float(np.arctan(normalized[0]))
                        if provisional and self.bearing_cue.observe(int(ident),timestamp,angle):
                            self.bearing_pub.publish(String(data=json.dumps(dict(id=int(ident),stamp=timestamp,angle=angle))))
                        continue
                    point=PointStamped(); point.header=msg.header
                    point.point.x,point.point.y,point.point.z=map(float,xyz)
                    # The map frame is fixed. A later map<-odom correction describes
                    # the robot's current drift and must not move old landmarks.
                    tr=self.tf.lookup_transform(self.p('map_frame'),frame,stamp,timeout=Duration(seconds=0.))
                    # Reject horizontal surfaces: task targets are on vertical walls/boxes.
                    if xyz is not None and dep is not None and abs((Time.from_msg(dep.header.stamp)-stamp).nanoseconds)<150_000_000 and dep.width==msg.width and dep.height==msg.height:
                        points=[]
                        for pixel in c[0]:
                            u,v=map(int,pixel)
                            patch=depth[max(0,v-2):v+3,max(0,u-2):u+3].astype(float)
                            if dep.encoding=='16UC1': patch*=.001
                            valid=patch[np.isfinite(patch)&(patch>.15)&(patch<8.)]
                            if valid.size<5: continue
                            z=float(np.median(valid)); q=PointStamped(); q.header=msg.header
                            q.point.x=(float(pixel[0])-k[2])/k[0]*z
                            q.point.y=(float(pixel[1])-k[5])/k[4]*z; q.point.z=z
                            pt=do_transform_point(q,tr).point; points.append([pt.x,pt.y,pt.z])
                        if len(points)>=3:
                            points=np.array(points); _,_,v=np.linalg.svd(points-points.mean(axis=0))
                            if abs(v[-1,2])>.5: self.reject(ident,'horizontal');continue
                    p=do_transform_point(point,tr).point
                    pos=np.array([p.x,p.y,p.z]); ident=int(ident)
                    if provisional:
                        # A candidate requests a closer view. It never enters the
                        # recognition history, targets, map output or approach log.
                        if self.provisional.observe(ident,pos,stamp.nanoseconds/1e9,float(edges.min())):
                            evidence=dict(stamp=stamp.nanoseconds/1e9,id=ident,map_point=pos.tolist(),
                                min_edge_pixels=float(edges.min()),provisional=True,
                                camera_origin_map=[tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z],
                                observation_frame=self.p('map_frame'))
                            self.candidate_pub.publish(String(data=json.dumps(evidence)))
                        continue
                    history=self.observations.setdefault(ident,[])
                    if not hasattr(self,'confirmation_guard'):self.confirmation_guard=ObservationConsensus()
                    verdict=self.confirmation_guard.update(ident,history,stamp.nanoseconds,pos,float(edges.min()),self.p('confirmations'))
                    if verdict in ('inconsistent','invalid'):self.reject(ident,'inconsistent');continue
                    if verdict=='duplicate':continue
                    if verdict=='recovered':
                        event=self.confirmation_guard.last_recovery
                        self.diagnostics.setdefault('seed_reacquisitions',[]).append(event)
                        recovered=len(event['readable_consensus'])
                        self.diagnostics['accepted']+=recovered-1
                        detail['accepted']=detail.get('accepted',0)+recovered-1
                    accepted_quads[ident]=c
                    self.diagnostics['accepted']+=1
                    detail['accepted']=detail.get('accepted',0)+1
                    if self.observation_pub.get_subscription_count():
                        evidence=dict(range_source=source,stamp=stamp.nanoseconds/1e9,id=ident,uv=uv.tolist(),camera_point=xyz.tolist(),
                                      min_edge_pixels=float(edges.min()),corners=c[0].tolist(),
                                      odom_point=None,map_point=pos.tolist(),lidar_camera_point=None,
                                      camera_origin_map=[tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z],
                                      observation_frame=self.p('map_frame'))
                        try:
                            odom=do_transform_point(point,self.tf.lookup_transform(self.p('odom_frame'),frame,stamp)).point
                            evidence['odom_point']=[odom.x,odom.y,odom.z]
                            lidar=self.lidar_point(ray,frame,stamp)
                            if lidar is not None:evidence['lidar_camera_point']=lidar.tolist()
                        except Exception as exc:evidence['audit_error']=str(exc)
                        self.observation_pub.publish(String(data=json.dumps(evidence)))
                except Exception as exc:
                    self.diagnostics['marker_errors']=self.diagnostics.get('marker_errors',0)+1
                    self.diagnostics['last_marker_error']=str(exc)
                    self.get_logger().debug(f'ID {int(ident)} skipped: {exc}')
            if accepted_quads:
                self.separate_markers.observe(stamp.nanoseconds,accepted_quads)
        except Exception as exc:
            self.diagnostics['errors']+=1;self.diagnostics['last_error']=str(exc)
            self.get_logger().debug(str(exc))
        finally:
            if processed is not None:self.processed_camera.publish(processed)

    def publish(self):
        ids,conflicts=resolved_ids(self.observations,self.p('confirmations'),separate=getattr(self,'separate_markers',None))
        self.diagnostic_pub.publish(String(data=json.dumps(dict(self.diagnostics,
            identity_conflicts=conflicts,resolved_ids=sorted(ids),
            by_id=self.id_diagnostics,
            exposure_queue=dict(self.exposure_queue.counts,pending=len(self.exposure_queue.rows)),
            pipeline=self.pipeline_timing.report(),sensor_history=self.sensor_history.report(),
            confirmations={str(k):len(v) for k,v in self.observations.items()}))))
        array=MarkerArray()
        try:
            clear=Marker();clear.ns='identity_reset';clear.action=Marker.DELETEALL
            array.markers.append(clear)
            for ident,history in self.observations.items():
                if ident not in ids: continue
                p=PointStamped(); p.header.frame_id=self.p('map_frame')
                p.point.x,p.point.y,p.point.z=map(float,np.median([v[1] for v in history],axis=0))
                for kind in ['point','label']:
                    m=Marker(); m.header.frame_id=self.p('map_frame'); m.header.stamp=self.get_clock().now().to_msg()
                    m.ns=kind; m.id=ident; m.action=Marker.ADD; m.pose.position=copy.deepcopy(p.point); m.pose.orientation.w=1.
                    m.color.r=1.; m.color.g=.25; m.color.a=1.
                    if kind=='point':
                        m.type=Marker.SPHERE; m.scale.x=m.scale.y=m.scale.z=.15
                    else:
                        m.type=Marker.TEXT_VIEW_FACING; m.text=f'ID {ident}'; m.scale.z=.25; m.pose.position.z+=.3
                    array.markers.append(m)
            self.pub.publish(array)
        except Exception as exc: self.get_logger().debug(str(exc))


def main(args=None):
    rclpy.init(args=args); node=Detector()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
