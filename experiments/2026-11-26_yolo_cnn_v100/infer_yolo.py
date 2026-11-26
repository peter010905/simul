"""Run the selected offline YOLO marker locator on an image or directory."""
import argparse
import json
from pathlib import Path

import cv2
from ultralytics import YOLO


def main():
    p=argparse.ArgumentParser();p.add_argument('--weights',type=Path,required=True)
    p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--device',default='0');p.add_argument('--confidence',type=float,default=.45)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    files=sorted(x for x in a.input.iterdir() if x.suffix.lower() in {'.jpg','.jpeg','.png'}) if a.input.is_dir() else [a.input]
    model=YOLO(str(a.weights));model.to(f'cuda:{a.device}' if a.device.isdigit() else a.device)
    records=[]
    for file in files:
        image=cv2.imread(str(file))
        if image is None:raise ValueError(file)
        result=model.predict(image,imgsz=640,conf=a.confidence,iou=.7,max_det=100,verbose=False)[0]
        hits=[];panel=image.copy()
        for xyxy,confidence in zip(result.boxes.xyxy.cpu().numpy(),result.boxes.conf.cpu().numpy()):
            x1,y1,x2,y2=map(float,xyxy)
            hits.append(dict(box_xyxy=[x1,y1,x2,y2],confidence=float(confidence),marker_id=None))
            cv2.rectangle(panel,(round(x1),round(y1)),(round(x2),round(y2)),(0,220,0),2)
            cv2.putText(panel,f'marker {confidence:.2f}',(round(x1),max(12,round(y1)-4)),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,220,0),1)
        destination=a.output/(file.stem+'-detections.jpg');cv2.imwrite(str(destination),panel)
        records.append(dict(input=str(file),image_size=[image.shape[1],image.shape[0]],detections=hits,preview=str(destination),
                            scope='visual location only; no ArUco ID or range'))
    (a.output/'detections.json').write_text(json.dumps(records,indent=2))
    print(json.dumps(dict(images=len(records),detections=sum(len(r['detections']) for r in records),output=str(a.output))),flush=True)


if __name__=='__main__':main()
