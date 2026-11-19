"""Exercise two-view confirmation against synchronized saved scan, map and camera poses."""
import sys, math, json
from pathlib import Path
from collections import Counter
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from tf2_ros import Buffer, TransformException
from rosidl_runtime_py.utilities import get_message
from osp_mission.two_views import TwoViews
from osp_mission.r32_drive import grid_from_message
from osp_mission.wall_route import yaw
from osp_mission.box_views import local_boxes, odom_point
from scipy import ndimage
from corner_pair import walking_distance

run=Path(sys.argv[1])
reader=SequentialReader()
reader.open(StorageOptions(uri=str(next(run.glob('native/*/drive_bag'))),storage_id='mcap'),ConverterOptions('',''))
names=['/global_costmap/costmap','/scan','/map','/tf','/tf_static','/oakd/rgb/preview/camera_info']
reader.set_filter(StorageFilter(topics=names))
types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in names}
tracker=TwoViews()
buf=Buffer()
scan=grid=pending=None
counts=Counter()
probes=[]
next_probe=0.
costmap=None
while reader.has_next():
    topic,data,_=reader.read_next()
    msg=deserialize_message(data,types[topic])
    if topic.startswith('/tf'):
        for t in msg.transforms:
            if topic=='/tf_static':buf.set_transform_static(t,'replay')
            else:buf.set_transform(t,'replay')
    elif topic=='/global_costmap/costmap':
        costmap=grid_from_message(msg)
    elif topic=='/map':
        grid=grid_from_message(msg)
    elif topic=='/scan':
        scan=msg
    else:
        pending=msg
    if pending is None or grid is None or scan is None:continue
    ct=Time.from_msg(pending.header.stamp)
    st=Time.from_msg(scan.header.stamp)
    stamp=ct.nanoseconds/1e9
    if stamp-tracker.last_frame<.4 or abs((st-ct).nanoseconds)>250000000:continue
    try:
        base=buf.lookup_transform('map',scan.header.frame_id,st).transform
        cam=buf.lookup_transform('map',pending.header.frame_id,ct).transform
        mt=buf.lookup_transform('odom','map',ct).transform
    except TransformException:
        continue
    rays=np.asarray(scan.ranges)
    ang=scan.angle_min+np.arange(len(rays))*scan.angle_increment+yaw(base.rotation)
    ok=np.isfinite(rays)&(rays>max(.25,scan.range_min))&(rays<scan.range_max)
    endpoints=np.column_stack((base.translation.x+rays[ok]*np.cos(ang[ok]),base.translation.y+rays[ok]*np.sin(ang[ok])))
    q=cam.rotation
    heading=math.atan2(2*(q.y*q.z-q.w*q.x),2*(q.x*q.z+q.w*q.y))
    fov=2*math.atan(pending.width/(2*pending.k[0]))
    tracker.observe(grid,(cam.translation.x,cam.translation.y,heading),endpoints,(mt.translation.x,mt.translation.y,yaw(mt.rotation)),stamp,fov)
    counts['frames']+=1
    pending=None
    if stamp<next_probe or costmap is None:
        continue
    next_probe=stamp+3
    pose=(cam.translation.x,cam.translation.y,heading)
    transform=(mt.translation.x,mt.translation.y,yaw(mt.rotation))
    goal=tracker.choose(grid,costmap,pose,transform,stamp,{},lambda g:False)
    if goal is None:
        continue
    view=np.array([goal['x'],goal['y']])
    center,faces=min(local_boxes(grid,pose),key=lambda item:np.linalg.norm(odom_point(item[0],transform)-goal['face_center_odom']))
    track=tracker.nearest(goal['face_center_odom'])
    visible=0
    for point,normal in faces:
        delta=view-point
        dist=np.linalg.norm(delta)
        angle=math.atan2(normal[1],normal[0])+transform[2]
        if any(math.cos(angle-a)>math.cos(math.radians(35)) for a in track.seen):
            continue
        aiming=math.atan2(-delta[1],-delta[0])
        if (0<dist<=1.45 and delta@normal>.4*dist and math.cos(aiming-goal['yaw'])>math.cos(min(fov/2-math.radians(4),math.radians(25)))
                and grid.line_clear(view,point+.12*normal)):
            visible+=1
    free=(grid.cells>=0)&(grid.cells<=25)
    clearance=ndimage.distance_transform_edt(np.pad(free,1))[1:-1,1:-1]*grid.resolution
    steps=walking_distance(clearance>=.32,grid.index(*pose[:2]),grid.resolution,8.)
    path=steps.get(grid.index(*view))
    probes.append(dict(t=round(stamp,2),faces=visible,walking_m=None if path is None else round(path*grid.resolution,2)))
print(json.dumps(dict(run=run.name,frames=counts['frames'],probes=probes),indent=2))
