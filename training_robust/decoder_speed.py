"""CPU-only synthetic ArUco decoder timing; excludes ROS, rendering and depth."""

import json
from pathlib import Path
import time

import cv2
import numpy as np


def main():
    cv2.setNumThreads(1)
    rng=np.random.default_rng(2026110551)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    result=dict(opencv=cv2.__version__,threads=1,environment='synthetic_cpu_decoder_only',
        includes_ros=False,includes_depth=False,includes_rendering=False,
        note='CPU headroom does not establish Gazebo real-time factor or real camera detection accuracy',rows=[])
    for width in [960,1280,1600]:
        images=[]
        for image_index in range(24):
            height=width*3//4
            image=rng.integers(170,220,(height,width),dtype=np.uint8)
            # Many geometric edges as well as three marker patches.
            for _ in range(60):
                x=int(rng.integers(width-100));y=int(rng.integers(height-100))
                cv2.rectangle(image,(x,y),(x+int(rng.integers(20,100)),y+int(rng.integers(20,100))),int(rng.integers(30,140)),2)
            ids=rng.choice(1024,3,replace=False).tolist()
            for index,ident in enumerate(ids):
                size=[24,40,80][index];x=50+index*(width//3);y=height//2
                image[y-8:y+size+8,x-8:x+size+8]=245
                image[y:y+size,x:x+size]=cv2.aruco.drawMarker(dictionary,ident,size)
            images.append((image,ids))
        for image,_ in images[:3]:cv2.aruco.detectMarkers(image,dictionary)
        timings=[];correct=0;expected=0
        for image,ids in images*3:
            start=time.perf_counter();_,detected,_=cv2.aruco.detectMarkers(image,dictionary)
            timings.append((time.perf_counter()-start)*1000)
            correct+=len(set(ids)&(set(detected.flatten()) if detected is not None else set()))
            expected+=len(ids)
        result['rows'].append(dict(width=width,height=width*3//4,frames=len(timings),
            median_ms=float(np.median(timings)),p95_ms=float(np.percentile(timings,95)),
            maximum_ms=max(timings),decoded=correct,expected=expected))
    out=Path('training_robust/decoder_speed.json');out.write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)


if __name__=='__main__':main()
