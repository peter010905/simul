"""Record real Gazebo camera topics beside the live SLAM map and laser scan.

No scenario/ground-truth input is accepted. Video runs at 1x simulation time;
held frames preserve elapsed time when rendering or callbacks are slow.
"""
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def label(image,text,x,y,scale=.55,color=(225,232,240)):
    cv2.putText(image,text,(x,y),cv2.FONT_HERSHEY_SIMPLEX,scale,color,1,cv2.LINE_AA)


def fit_image(canvas,source,rect,title):
    x,y,w,h=rect
    canvas[y:y+h,x:x+w]=(40,30,24)
    if source is not None:
        scale=min(w/source.shape[1],(h-28)/source.shape[0])
        frame=cv2.resize(source,(max(1,round(source.shape[1]*scale)),max(1,round(source.shape[0]*scale))))
        yy=y+28+(h-28-frame.shape[0])//2;xx=x+(w-frame.shape[1])//2
        canvas[yy:yy+frame.shape[0],xx:xx+frame.shape[1]]=frame
    else:label(canvas,'Waiting for actual sensor frames',x+12,y+h//2,.5,(80,180,255))
    label(canvas,title,x+10,y+19,.49)


def map_panel(grid=None,resolution=.05,origin=(0.,0.,0.),robot=None,scan=None,path=(),targets=()):
    panel=np.full((626,460,3),(40,30,24),np.uint8)
    label(panel,'LIVE SLAM + LiDAR',12,24,.6)
    label(panel,'Cyan: laser hits   Green: traveled path',12,48,.43)
    label(panel,'Dark blue: unknown   White: mapped free',12,68,.43)
    if grid is None:
        label(panel,'Waiting for /map',12,300)
        return panel
    height,width=grid.shape
    scale=min(436/width,512/height)
    size=(max(1,round(width*scale)),max(1,round(height*scale)))
    left=(460-size[0])//2;top=86+(512-size[1])//2
    colors=np.full((height,width,3),(61,48,35),np.uint8)
    colors[(grid>=0)&(grid<65)]=(220,224,225)
    colors[grid>=65]=(22,24,27)
    panel[top:top+size[1],left:left+size[0]]=cv2.resize(colors[::-1],size,interpolation=cv2.INTER_NEAREST)
    ox,oy,angle=origin;c,s=math.cos(angle),math.sin(angle)
    def pixel(x,y):
        dx,dy=x-ox,y-oy
        gx=(c*dx+s*dy)/resolution;gy=(-s*dx+c*dy)/resolution
        return (round(left+gx*size[0]/width),round(top+(height-gy)*size[1]/height))
    if len(path)>1:
        cv2.polylines(panel,[np.array([pixel(*p) for p in path],np.int32)],False,(76,163,45),2,cv2.LINE_AA)
    if scan is not None:
        sensor,points=scan
        start=pixel(*sensor)
        for p in points[::12]:cv2.line(panel,start,pixel(*p),(115,120,76),1,cv2.LINE_AA)
        for p in points:cv2.circle(panel,pixel(*p),1,(240,206,20),-1)
    if robot is not None:
        x,y,a=robot;p=pixel(x,y)
        cv2.circle(panel,p,6,(80,70,245),-1)
        cv2.arrowedLine(panel,p,pixel(x+.45*math.cos(a),y+.45*math.sin(a)),(80,70,245),2)
    for ident,x,y in targets:
        p=pixel(x,y);cv2.circle(panel,p,7,(0,170,255),2)
        label(panel,str(ident),p[0]+8,p[1]-6,.5,(0,160,255))
    known=np.count_nonzero(grid>=0)*resolution**2
    label(panel,f'Mapped area: {known:.1f} m2',12,618,.49)
    return panel


class SimulationVideo:
    """Constant-rate MP4 that holds prior frames across timestamp gaps."""
    def __init__(self,directory,fps=5.):
        self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=True)
        self.path=self.directory/'mission.mp4'
        if self.path.exists():raise FileExistsError(self.path)
        self.fps=fps;self.writer=None;self.start=None;self.frames=0;self.samples=0
        self.last_frame=None;self.last_stamp=None;self.closed=False;self.held_frames=0
        self.table=(self.directory/'frames.csv').open('w',newline='')
        self.rows=csv.writer(self.table)
        self.rows.writerow(['frame','video_seconds','source_sim_seconds','held_previous_frame'])

    def write(self,frame,stamp):
        if self.closed:raise RuntimeError('Recording is closed')
        if self.start is None:
            self.start=stamp
            self.writer=cv2.VideoWriter(str(self.path),cv2.VideoWriter_fourcc(*'avc1'),self.fps,(1280,720))
            if not self.writer.isOpened():raise RuntimeError('H.264 encoder is unavailable')
            cv2.imwrite(str(self.directory/'first_frame.png'),frame)
        if frame.shape!=(720,1280,3):raise ValueError('Expected a 1280x720 BGR frame')
        if self.last_stamp is not None and stamp<self.last_stamp:raise ValueError('Simulation clock reset during recording')
        target=round((stamp-self.start)*self.fps)
        if target<self.frames:return
        while self.frames<target:
            self._write(self.last_frame,self.last_stamp,True)
        self._write(frame,stamp,False)
        self.last_frame=frame.copy();self.last_stamp=stamp;self.samples+=1
        # Keep ongoing runs inspectable without waiting for MP4 finalization.
        if self.samples%25==0:
            self.table.flush()
            cv2.imwrite(str(self.directory/'latest_frame.png'),frame)

    def _write(self,frame,stamp,held):
        self.writer.write(frame)
        self.rows.writerow([self.frames,self.frames/self.fps,stamp,int(held)])
        self.frames+=1;self.held_frames+=int(held)

    def close(self,metadata=None):
        if self.closed:return
        self.closed=True
        if self.writer is not None:self.writer.release()
        if self.last_frame is not None:cv2.imwrite(str(self.directory/'last_frame.png'),self.last_frame)
        self.table.close()
        report={'video':str(self.path) if self.frames else None,'frames':self.frames,'fps':self.fps,
            'duration_seconds':self.frames/self.fps,'composite_samples':self.samples,'held_frames':self.held_frames,
            'start_sim_seconds':self.start,'last_sim_seconds':self.last_stamp,
            'playback':'1x simulation time; stale inputs/held frames are not interpolated',
            'sources':['Gazebo observer camera','robot RGB camera','ROS /map','ROS /scan','ROS TF','ROS /targets'],
            **(metadata or {})}
        (self.directory/'recording.json').write_text(json.dumps(report,indent=2))


class Recorder(Node):
    def __init__(self):
        super().__init__('mission_video_recorder')
        self.declare_parameter('output_dir',str(Path.home()/'simul/results/video'))
        self.video=SimulationVideo(self.get_parameter('output_dir').value)
        self.bridge=CvBridge();self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.images={};self.counts={k:0 for k in ['overview','robot_rgb','map','scan']}
        self.grid=None;self.scan=None;self.targets=[];self.path=[];self.status={};self.error=None
        self.first_record_sim=None;self.first_counts=None
        latest_image=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image,'/mission/overview/image_raw',lambda m:self.on_image('overview',m),latest_image)
        self.create_subscription(Image,'oakd/rgb/preview/image_raw',lambda m:self.on_image('robot_rgb',m),latest_image)
        map_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid,'map',self.on_map,map_qos)
        self.create_subscription(LaserScan,'scan',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(MarkerArray,'targets',self.on_targets,10)
        self.create_subscription(String,'mission/status',self.on_status,10)
        self.create_timer(.2,self.capture)

    def on_image(self,key,message):
        try:
            self.images[key]=(self.bridge.imgmsg_to_cv2(message,'bgr8'),Time.from_msg(message.header.stamp).nanoseconds/1e9)
            self.counts[key]+=1
        except Exception as exc:self.get_logger().warning(f'{key}: {exc}')

    def on_map(self,message):self.grid=message;self.counts['map']+=1
    def on_scan(self,message):self.scan=message;self.counts['scan']+=1
    def on_targets(self,message):
        self.targets=[(m.id,m.pose.position.x,m.pose.position.y) for m in message.markers if m.ns=='point' and m.header.frame_id=='map']
    def on_status(self,message):
        self.status=json.loads(message.data)
        if self.status.get('state') in ('saving','done'):
            self.capture(final=True);self.close()

    def capture(self,final=False):
        if self.video.closed or self.status.get('state')!='running' and not final:return
        # Never create a "simulation video" without actual rendered image data.
        if not self.images:return
        now=self.get_clock().now().nanoseconds/1e9
        if self.first_counts is None:self.first_counts=dict(self.counts);self.first_record_sim=now
        frame=np.full((720,1280,3),(30,22,17),np.uint8)
        label(frame,'GAZEBO | Robot search + live LiDAR / SLAM',16,29,.74)
        label(frame,f'Sim {now:.1f}s  |  1x simulation time',900,28,.48)
        for key,rect,title in [('overview',(12,44,784,438),'Gazebo observer camera'),
                               ('robot_rgb',(12,494,310,214),'Robot RGB camera')]:
            entry=self.images.get(key);age=now-entry[1] if entry else None
            suffix=f' | age {age:.1f}s' if age is not None else ''
            if age is not None and age>1.:suffix+=' STALE'
            fit_image(frame,entry[0] if entry else None,rect,title+suffix)
        robot=None;scan_points=None
        try:
            tr=self.tf.lookup_transform('map','base_link',Time()).transform
            robot=(tr.translation.x,tr.translation.y,yaw(tr.rotation))
            if not self.path or math.dist(self.path[-1],robot[:2])>.03:self.path.append(robot[:2])
        except Exception:pass
        if self.scan is not None and now-Time.from_msg(self.scan.header.stamp).nanoseconds/1e9<=1.:
            try:
                m=self.scan;tr=self.tf.lookup_transform('map',m.header.frame_id,Time.from_msg(m.header.stamp)).transform
                ranges=np.asarray(m.ranges);angles=m.angle_min+np.arange(len(ranges))*m.angle_increment+yaw(tr.rotation)
                valid=np.isfinite(ranges)&(ranges>=m.range_min)&(ranges<=m.range_max)
                points=np.column_stack((tr.translation.x+ranges[valid]*np.cos(angles[valid]),tr.translation.y+ranges[valid]*np.sin(angles[valid])))
                scan_points=((tr.translation.x,tr.translation.y),points)
            except Exception:pass
        kwargs={'robot':robot,'scan':scan_points,'path':self.path,'targets':self.targets}
        if self.grid is not None:
            info=self.grid.info
            kwargs.update(grid=np.asarray(self.grid.data).reshape(info.height,info.width),resolution=info.resolution,
                origin=(info.origin.position.x,info.origin.position.y,yaw(info.origin.orientation)))
        frame[44:670,808:1268]=map_panel(**kwargs)
        for i,line in enumerate([
            f"Mission: {self.status.get('state','waiting')}",
            f"Elapsed: {self.status.get('elapsed_seconds',0.):.1f} sim seconds",
            f"Confirmed targets: {self.status.get('targets',0)} / 3",
            f"Contacts: {self.status.get('collisions',0)}",
            'Playback: 1x simulation time',
            'Orange circles: confirmed marker IDs',
            'Unknown space appears dark blue.'
        ]):label(frame,line,336,516+i*27,.48)
        health='Laser TF: current' if scan_points is not None else 'Laser overlay: waiting / stale'
        if robot is None:health+=' | Robot TF missing'
        label(frame,health,816,699,.43,(110,200,240))
        try:self.video.write(frame,now)
        except Exception as exc:
            self.error=str(exc);self.get_logger().error(self.error);self.close()

    def close(self):
        self.video.close({'last_mission_status':self.status,'received_messages':self.counts,
            'messages_before_first_frame':self.first_counts,'error':self.error,
            'complete_visual_sources':all(self.counts.values())})


def main(args=None):
    rclpy.init(args=args);node=Recorder()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
