"""Offline check on recorded MP4s. Unmatched squares are not labeled positive markers."""
import argparse
import json
import math
import cv2
from osp_mission.thorough_borders import borders

def audit(path):
    cap=cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise OSError(path)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    detect=(cv2.aruco.ArucoDetector(dictionary).detectMarkers if hasattr(cv2.aruco,'ArucoDetector')
            else lambda im:cv2.aruco.detectMarkers(im,dictionary))
    count=raw=unmatched=0
    while True:
        ok,img=cap.read()
        if not ok:
            break
        gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        hits=borders(gray)
        count+=1
        if not hits:
            continue
        raw+=1
        corners,_,_=detect(gray)
        if any(not any(math.dist(c.reshape(-1,2).mean(axis=0),(u,v))<.45*s for c in corners)
               for u,v,s in hits):
            unmatched+=1
    cap.release()
    return dict(frames=count,with_outlines=raw,with_outline_without_matching_standard_decode=unmatched)

if __name__=='__main__':
    from pathlib import Path
    p=argparse.ArgumentParser()
    p.add_argument('files',type=Path,nargs='+')
    args=p.parse_args()
    cv2.setNumThreads(1)
    print(json.dumps({f.parent.name:audit(f) for f in args.files},indent=2))
