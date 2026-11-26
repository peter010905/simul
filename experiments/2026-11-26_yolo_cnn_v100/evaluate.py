"""Evaluate all four models on identical native frames at IoU 0.50."""
import argparse
from collections import Counter,defaultdict
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import torch


def iou(a,b):
    x,y,w,h=a;u,v,s,t=b
    area=max(0,min(x+w,u+s)-max(x,u))*max(0,min(y+h,v+t)-max(y,v))
    return area/max(1e-9,w*h+s*t-area)


def nms(hits,cutoff=.3,limit=100):
    kept=[]
    for hit in sorted(hits,key=lambda x:x['score'],reverse=True):
        if all(iou(hit['box'],row['box'])<=cutoff for row in kept):kept.append(hit)
        if len(kept)>=limit:break
    return kept


def match(hits,gt,threshold):
    hits=[h for h in hits if h['score']>=threshold]
    pairs=sorted(((iou(h['box'],g),i,j) for i,h in enumerate(hits) for j,g in enumerate(gt)),reverse=True)
    used_h=set();used_g=set()
    for score,i,j in pairs:
        if score<.5:break
        if i not in used_h and j not in used_g:used_h.add(i);used_g.add(j)
    return dict(tp=len(used_g),fp=len(hits)-len(used_h),fn=len(gt)-len(used_g),matched_gt=sorted(used_g))


def infer_cnn(model,gray,device,score_floor=.05):
    from candidate import proposals,crop
    regions,_=proposals(gray)
    hits=[]
    with torch.inference_mode():
        for first in range(0,len(regions),512):
            sub=regions[first:first+512]
            batch=np.stack([crop(gray,r.box) for r in sub])
            x=torch.from_numpy(batch[:,None].copy()).to(device,dtype=torch.float32)/255
            prob=torch.sigmoid(model(x)).cpu().numpy()
            for region,score in zip(sub,prob):
                if score>=score_floor:hits.append(dict(box=list(region.box),score=float(score)))
    return nms(hits)


def infer_yolo(model,bgr):
    result=model.predict(bgr,imgsz=640,conf=.001,iou=.7,max_det=100,verbose=False)[0]
    hits=[]
    for xyxy,confidence in zip(result.boxes.xyxy.cpu().numpy(),result.boxes.conf.cpu().numpy()):
        x,y,x2,y2=map(float,xyxy)
        hits.append(dict(box=[x,y,x2-x,y2-y],score=float(confidence)))
    return hits


def score(rows,threshold):
    overall=Counter();by_tag=defaultdict(Counter);frame_ms=[]
    for row in rows:
        found=match(row['hits'],row['gt'],threshold)
        overall.update({k:found[k] for k in ('tp','fp','fn')})
        overall['frames']+=1;overall['gt']+=len(row['gt'])
        overall['negative_frames']+=int(not row['gt'])
        overall['false_positive_negative_frames']+=found['fp'] if not row['gt'] else 0
        for tag in set(row['tags']):
            if row['gt']:
                by_tag[tag]['gt']+=len(row['gt']);by_tag[tag]['found']+=len(found['matched_gt'])
        frame_ms.append(row['ms'])
    tp,fp,fn=(overall[k] for k in ('tp','fp','fn'))
    overall['precision']=tp/max(1,tp+fp);overall['recall']=tp/max(1,tp+fn)
    overall['f1']=2*tp/max(1,2*tp+fp+fn)
    overall['median_ms']=float(np.median(frame_ms));overall['p95_ms']=float(np.percentile(frame_ms,95))
    return dict(overall=overall,by_tag={k:dict(v,recall=v['found']/max(1,v['gt'])) for k,v in by_tag.items()})


def main():
    p=argparse.ArgumentParser();p.add_argument('--prepared',type=Path,required=True);p.add_argument('--weights',type=Path,required=True)
    p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--kind',choices=['yolo','cnn'],required=True);p.add_argument('--variant',choices=['plain','augmented'],required=True)
    p.add_argument('--split',choices=['val','test'],required=True);p.add_argument('--device',type=int,default=0)
    p.add_argument('--calibration',type=Path)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    sys.path.insert(0,str(a.candidate));cv2.setNumThreads(2);torch.set_num_threads(2)
    if a.kind=='yolo':
        from ultralytics import YOLO
        model=YOLO(str(a.weights));model.to(f'cuda:{a.device}')
    else:
        from candidate import CandidateCNN
        model=CandidateCNN().to(f'cuda:{a.device}')
        model.load_state_dict(torch.load(a.weights,map_location=f'cuda:{a.device}',weights_only=True));model.eval()
    root=a.prepared/'plain'/a.split
    metadata=json.loads((a.prepared.parent/'data/annotations/instances_val2017.json').read_text())
    meta={x['file_name']:x for x in metadata['images']}
    rows=[]
    with (a.output/'frames.jsonl').open('w') as file:
        for i,image_path in enumerate(sorted((root/'images').glob('*.jpg'))):
            bgr=cv2.imread(str(image_path))
            start=time.perf_counter()
            hits=infer_yolo(model,bgr) if a.kind=='yolo' else infer_cnn(model,cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY),f'cuda:{a.device}')
            if torch.cuda.is_available():torch.cuda.synchronize(a.device)
            duration=1000*(time.perf_counter()-start)
            gt=[]
            for line in (root/'labels'/image_path.with_suffix('.txt').name).read_text().splitlines():
                _,cx,cy,w,h=map(float,line.split());gt.append([(cx-w/2)*640,(cy-h/2)*480,w*640,h*480])
            key=image_path.name.removeprefix('val_')
            row=dict(file=image_path.name,gt=gt,tags=meta[key].get('difficulty_tags',[]),hits=hits,ms=duration)
            rows.append(row);file.write(json.dumps(row)+'\n')
            if i%100==0:print(a.kind,a.variant,a.split,i+1,'/',len(list((root/'images').glob('*.jpg'))),flush=True)
    if a.split=='val':
        grid=list(np.arange(.05,.96,.05))+[.97,.98,.99,.995,.999,.9995,.9999,.99995,.99999]
        entries=[dict(threshold=float(t),**score(rows,float(t))) for t in grid]
        winner=max(entries,key=lambda x:x['overall']['f1'])
        (a.output/'thresholds.json').write_text(json.dumps(entries,indent=2))
        summary=dict(kind=a.kind,variant=a.variant,split='calibration_500',selected=winner)
    else:
        if not a.calibration:raise ValueError('--calibration required for sealed test')
        threshold=json.loads(a.calibration.read_text())['selected']['threshold']
        summary=dict(kind=a.kind,variant=a.variant,split='held_out_1000',threshold=threshold,**score(rows,threshold))
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary),flush=True)


if __name__=='__main__':main()
