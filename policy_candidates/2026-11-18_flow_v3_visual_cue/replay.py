"""Replay RGB in acquisition order, report cue frames for visual review."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py as bag
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from osp_mission.marker_pair import decode
from osp_mission.visual_cue import VisualCue


def replay(run, directory):
    reader = bag.SequentialReader()
    reader.open(bag.StorageOptions(uri=str(run/'rosbag'), storage_id='mcap'), bag.ConverterOptions('',''))
    rgb = '/oakd/rgb/preview/image_raw'
    info = '/oakd/rgb/preview/camera_info'
    types = {v.name:get_message(v.type) for v in reader.get_all_topics_and_types() if v.name in (rgb,info)}
    reader.set_filter(bag.StorageFilter(topics=list(types)))
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    tracker = VisualCue()
    bridge = CvBridge()
    calibration = first = None
    events, first_ids, tiles = [], {}, []
    frames = 0
    while reader.has_next():
        topic,data,_ = reader.read_next()
        msg = deserialize_message(data,types[topic])
        if topic == info:
            calibration = msg
            continue
        if calibration is None or calibration.k[0]<=0:
            continue
        image = bridge.imgmsg_to_cv2(msg,'mono8')
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if first is None:
            first = stamp
        frames += 1
        quads = []
        found, ids = decode(image,dictionary,quads)
        readable = []
        for q, ident in zip(found, ids.ravel() if ids is not None else ()):
            if np.linalg.norm(q[0]-np.roll(q[0],1,axis=0),axis=1).min()>=18:
                readable.append(int(ident))
                first_ids.setdefault(int(ident),round(stamp-first,2))
        cue = tracker.update(image,quads,readable,stamp,calibration.k[0],calibration.k[2])
        if cue:
            xy = tuple(int(round(v)) for v in cue['uv'])
            t = round(stamp-first,2)
            events.append([t,*xy,round(cue['min_edge_pixels'],1)])
            tile=cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
            cv2.circle(tile,xy,int(cue['min_edge_pixels']*.85),(0,160,255),2)
            cv2.putText(tile,str(t),(2,18),0,.55,(0,180,255),1)
            tiles.append(tile)
    print(json.dumps(dict(run=run.name,frames=frames,seconds=round(stamp-first,2),first_ids=first_ids,cues=events)),flush=True)
    if tiles:
        while len(tiles)%4:tiles.append(np.zeros_like(tiles[0]))
        cv2.imwrite(str(directory/(run.name+'.jpg')),np.vstack([np.hstack(tiles[i:i+4]) for i in range(0,len(tiles),4)]))


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    parser.add_argument('runs',nargs='+',type=Path)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    cv2.setNumThreads(1)
    for run in args.runs:
        replay(run,args.output)
