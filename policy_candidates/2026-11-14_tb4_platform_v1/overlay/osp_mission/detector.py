"""ArUco observations anchored in the map at the camera exposure time."""
import math
import copy
import json
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
        self.create_subscription(Image,self.p('rgb_topic'),self.on_rgb,latest_image)
        self.dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.create_timer(.5,self.publish)

    def on_info(self,m): self.info=m
    def reject(self, ident, reason):
        self.diagnostics[reason]+=1
        detail=self.id_diagnostics.setdefault(str(int(ident)),{})
        detail[reason]=detail.get(reason,0)+1
    def on_scan(self,m): self.scan=m
    def on_depth(self,m): self.depth=m

    def lidar_point(self, ray, frame, stamp):
        scan=self.scan
        if scan is None or abs((Time.from_msg(scan.header.stamp)-stamp).nanoseconds)>250_000_000:
            return None
        tr=self.tf.lookup_transform(scan.header.frame_id,frame,stamp,timeout=Duration(seconds=.05))
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

    def on_rgb(self,msg):
        self.diagnostics['frames']+=1
        if self.info is None: return
        try:
            image=self.bridge.imgmsg_to_cv2(msg,'mono8')
            k=self.info.k
            if not np.isfinite(k).all() or k[0]<=0 or k[4]<=0 or self.info.width!=msg.width or self.info.height!=msg.height: return
            corners,ids,_=cv2.aruco.detectMarkers(image,self.dictionary)
            processed=copy.deepcopy(self.info);processed.header=msg.header
            self.processed_camera.publish(processed)
            if ids is None: return
            # CameraInfo distortion is applied before back projection.
            for c,ident in zip(corners,ids.flatten()):
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
                xyz=None; range_source='none'; stamp=Time.from_msg(msg.header.stamp)
                dep=self.depth
                if dep is not None and abs((Time.from_msg(dep.header.stamp)-stamp).nanoseconds)<150_000_000 and dep.width==msg.width and dep.height==msg.height:
                    # Registered depth only; sim uses the same RGBD sensor.
                    depth=self.bridge.imgmsg_to_cv2(dep,'passthrough')
                    u,v=map(int,uv); patch=depth[max(0,v-2):v+3,max(0,u-2):u+3].astype(float)
                    if dep.encoding=='16UC1': patch*=.001
                    valid=patch[np.isfinite(patch)&(patch>.15)&(patch<8.)]
                    if valid.size>=5: xyz=ray*float(np.median(valid));range_source='registered_depth'
                frame=msg.header.frame_id
                if xyz is None:
                    xyz=self.lidar_point(ray,frame,stamp);range_source='lidar_plane'
                if xyz is None: self.reject(ident,'no_range');continue
                point=PointStamped(); point.header=msg.header
                point.point.x,point.point.y,point.point.z=map(float,xyz)
                # The map frame is fixed. A later map<-odom correction describes
                # the robot's current drift and must not move old landmarks.
                tr=self.tf.lookup_transform(self.p('map_frame'),frame,stamp,timeout=Duration(seconds=.05))
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
                self.diagnostics['accepted']+=1
                detail['accepted']=detail.get('accepted',0)+1
                if self.observation_pub.get_subscription_count():
                    evidence=dict(stamp=stamp.nanoseconds/1e9,id=ident,uv=uv.tolist(),camera_point=xyz.tolist(),
                                  range_source=range_source,independent_depth_lidar=(range_source=='registered_depth'),
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
            self.diagnostics['errors']+=1;self.diagnostics['last_error']=str(exc)
            self.get_logger().debug(str(exc))

    def publish(self):
        ids,conflicts=resolved_ids(self.observations,self.p('confirmations'))
        self.diagnostic_pub.publish(String(data=json.dumps(dict(self.diagnostics,
            identity_conflicts=conflicts,resolved_ids=sorted(ids),
            by_id=self.id_diagnostics,
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
