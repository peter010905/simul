"""Check anonymous observations against recorded ranges, known-space costmaps and ID observations."""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace as NS

import cv2
import numpy as np
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer
from osp_mission.detector import Detector
from osp_mission.hybrid import PrintMemory,grid_from_message,publish_print,yaw
from osp_mission.marker_pair import decode


def stream(run,names):
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(run/'rosbag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in names}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    while reader.has_next():
        k,b,ns=reader.read_next()
        yield k,deserialize_message(b,types[k]),ns/1e9


def audit(run,target):
    start=next(e['wall_epoch_s'] for e in map(json.loads,(run/'artifacts/startup.jsonl').read_text().splitlines()) if e['event']=='mission_started')
    cutoff=None; ref=None
    for _,m,t in stream(run,['/mission/marker_observations']):
        e=json.loads(m.data)
        if e['id']==target and e.get('odom_point') is not None and t>=start:
            ref=np.array(e['odom_point'][:2]);break
    tf=Buffer(cache_time=Duration(seconds=3600))
    for k,m,_ in stream(run,['/tf','/tf_static']):
        for tr in m.transforms:
            (tf.set_transform if k=='/tf' else tf.set_transform_static)(tr,'recorded')
    node=NS(tf=tf,p=lambda k:'odom',scan=None,info=None)
    node.lidar_point=lambda *args:Detector.lidar_point(node,*args)
    memory=PrintMemory()
    reports=[]; matched=[]; proposals=[];errors={}; last_check=-1; cost=grid=None; gate=None
    def received(msg):
        e=json.loads(msg.data)
        memory.observe(e,t)
        reports.append(e)
        if ref is not None and np.linalg.norm(ref-e['point'])<=.30:
            matched.append(round(t-start,3))
    node.print_pub=NS(publish=received)
    camera='/oakd/rgb/preview/'
    bridge=CvBridge();dic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    for k,m,t in stream(run,[camera+'image_raw',camera+'camera_info','/map','/global_costmap/costmap','/scan_self_filtered']):
        if k=='/map':grid=grid_from_message(m);continue
        if k.endswith('/costmap'):
            cost=grid_from_message(m);cost_time=Time.from_msg(m.header.stamp).nanoseconds/1e9;continue
        if k.endswith('camera_info'):node.info=m;continue
        if k.startswith('/scan'):node.scan=m;continue
        if t<start or node.info is None:continue
        image=bridge.imgmsg_to_cv2(m,'mono8');qs,ids=decode(image,dic)
        if ids is not None:
            if any(i==target and np.linalg.norm(q[0]-np.roll(q[0],1,axis=0),axis=1).min()>=18 for q,i in zip(qs,ids.ravel())):
                cutoff=round(t-start,3);break
        try:
            if ids is None:publish_print(node,m,image)
            if t-last_check<.5 or grid is None or cost is None or not -.1<=t-cost_time<=3:continue
            last_check=t
            stamp=Time.from_msg(m.header.stamp)
            pose=tf.lookup_transform('map','base_link',stamp).transform
            trans=tf.lookup_transform('odom','map',stamp).transform
            goal=memory.choose(grid,cost,(pose.translation.x,pose.translation.y,yaw(pose.rotation)),
                    (trans.translation.x,trans.translation.y,yaw(trans.rotation)),{},t,lambda g:False)
            if goal:
                dist=float(np.linalg.norm(ref-goal['print_odom'])) if ref is not None else None
                proposals.append(dict(sec=round(t-start,3),target_error_m=dist,
                    robot_to_goal_m=goal['distance'],stationary=goal['stationary']))
                memory.used.append((np.array(goal['print_odom']),t))
        except Exception as exc:
            name=type(exc).__name__;errors[name]=errors.get(name,0)+1
    return dict(run=run.name,id=target,first_decode_s=cutoff,first_spatial_match_s=matched[:1],
                matched_frames=len(set(matched)),proposals=proposals,errors=errors)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path)
    a=p.parse_args();cv2.setNumThreads(1)
    print(json.dumps([audit(a.root/run,i) for run,i in [('20261118T083739-28a113496c',7),
        ('20261118T082353-9f8e401d05',7),('20261118T035817-a943a066df',7)]],indent=2))
