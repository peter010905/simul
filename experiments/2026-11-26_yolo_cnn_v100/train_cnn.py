"""Prepare and train matched geometry-free CNN marker proposal classifiers."""
import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import torch
from torch.nn import functional as F


def read_boxes(label):
    boxes=[]
    for line in label.read_text().splitlines():
        _,cx,cy,w,h=map(float,line.split())
        boxes.append(((cx-w/2)*640,(cy-h/2)*480,w*640,h*480))
    return boxes


def intersection(a,b):
    x,y,w,h=a;u,v,s,t=b
    return max(0,min(x+w,u+s)-max(x,u))*max(0,min(y+h,v+t)-max(y,v))


def prepare_group(source,output,rng,files=None):
    from candidate import proposals,crop,box_iou
    images=[];labels=[];stats={'frames':0,'boxes':0,'boxes_with_contour':0,'positive_patches':0,'negative_patches':0}
    files=sorted((source/'images').glob('*.jpg')) if files is None else files
    for i,file in enumerate(files):
        gray=cv2.imread(str(file),cv2.IMREAD_GRAYSCALE)
        if gray is None or gray.shape!=(480,640):raise ValueError(file)
        gt=read_boxes(source/'labels'/file.with_suffix('.txt').name)
        regions,_=proposals(gray)
        stats['frames']+=1;stats['boxes']+=len(gt)
        used=set()
        for box in gt:
            matches=sorted(((box_iou(r.box,box),j) for j,r in enumerate(regions)),reverse=True)
            if matches and matches[0][0]>=.35:
                j=matches[0][1];used.add(j)
                images.append(crop(gray,regions[j].box));labels.append(1)
                stats['boxes_with_contour']+=1;stats['positive_patches']+=1
        safe=[]
        for j,r in enumerate(regions):
            if j in used:continue
            if all(intersection(r.box,(b[0]-8,b[1]-8,b[2]+16,b[3]+16))<=1 for b in gt):safe.append(j)
        for j in rng.choice(safe,min(3,len(safe)),replace=False):
            images.append(crop(gray,regions[int(j)].box));labels.append(0)
            stats['negative_patches']+=1
        if i%1000==0:print(output.name,i+1,'/',len(files),flush=True)
    output.parent.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(output,images=np.stack(images),labels=np.asarray(labels,np.uint8))
    output.with_suffix('.json').write_text(json.dumps(stats,indent=2))
    print(output,stats,flush=True)


def scores(model,images,device):
    model.eval();out=[]
    with torch.inference_mode():
        for i in range(0,len(images),512):
            batch=torch.from_numpy(images[i:i+512,None].copy()).to(device,dtype=torch.float32)/255
            out.extend(torch.sigmoid(model(batch)).cpu().tolist())
    return np.asarray(out)


def f1(labels,prob,threshold):
    hit=prob>=threshold;positive=labels==1
    tp=int((hit&positive).sum());fp=int((hit&~positive).sum());fn=int((~hit&positive).sum())
    return 2*tp/max(1,2*tp+fp+fn)


def train(prepared,output,variant,device,epochs,steps):
    from candidate import CandidateCNN
    torch.manual_seed(261126);rng=np.random.default_rng(261126)
    def unpack(path):
        # NpzFile lazily decompresses on each indexing operation; keep the
        # arrays in memory so a training step never reopens a 36 MB archive.
        with np.load(path) as packed:
            return {'images':packed['images'],'labels':packed['labels']}
    plain=unpack(output/'cnn-patches/plain.npz')
    aug=unpack(output/'cnn-patches/augmented.npz') if variant=='augmented' else None
    val=unpack(output/'cnn-patches/val.npz')
    datasets=[plain] if aug is None else [plain,aug]
    pools=[]
    for data in datasets:pools.append({label:np.flatnonzero(data['labels']==label) for label in (0,1)})
    model=CandidateCNN().to(device);opt=torch.optim.AdamW(model.parameters(),lr=3e-4,weight_decay=1e-4)
    best=-1;history=[];folder=output/'cnn'/variant;folder.mkdir(parents=True,exist_ok=True)
    for epoch in range(epochs):
        model.train();losses=[]
        for _ in range(steps):
            source=int(rng.choice(len(datasets)))
            ix=np.concatenate([rng.choice(pools[source][0],64,replace=True),rng.choice(pools[source][1],64,replace=True)])
            rng.shuffle(ix)
            x=torch.from_numpy(datasets[source]['images'][ix,None].copy()).to(device,dtype=torch.float32)/255
            y=torch.from_numpy(datasets[source]['labels'][ix].copy()).to(device,dtype=torch.float32)
            loss=F.binary_cross_entropy_with_logits(model(x),y)
            opt.zero_grad();loss.backward();opt.step();losses.append(float(loss.detach()))
        probability=scores(model,val['images'],device)
        thresholds=np.arange(.05,1,.05)
        winner=max(((f1(val['labels'],probability,t),float(t)) for t in thresholds),key=lambda x:x[0])
        entry={'epoch':epoch+1,'loss':float(np.mean(losses)),'validation_patch_f1':winner[0],'validation_threshold':winner[1]}
        history.append(entry);(folder/'history.json').write_text(json.dumps(history,indent=2))
        if winner[0]>best:
            best=winner[0];torch.save(model.state_dict(),folder/'best.pth');(folder/'selection.json').write_text(json.dumps(entry,indent=2))
        print(variant,entry,flush=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('--prepared',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--candidate',type=Path,required=True);p.add_argument('--stage',choices=['prepare','train'],required=True)
    p.add_argument('--variant',choices=['plain','augmented']);p.add_argument('--device',type=int,default=2)
    p.add_argument('--epochs',type=int,default=10);p.add_argument('--steps',type=int,default=500)
    a=p.parse_args();sys.path.insert(0,str(a.candidate));cv2.setNumThreads(2);torch.set_num_threads(2)
    if a.stage=='prepare':
        rng=np.random.default_rng(261126)
        groups={'plain':a.prepared/'plain/train','augmented':a.prepared/'augmented/train','val':a.prepared/'plain/val'}
        for name,source in groups.items():
            if name=='augmented':
                # Only the added transformed images; originals are in plain.
                source=source
            output=a.output/'cnn-patches'/f'{name}.npz'
            if output.exists():continue
            if name=='augmented':
                files=sorted((source/'images').glob('aug_*.jpg'))
                # prepare_group accepts an explicit subset by using a temporary list below.
                prepare_group(source,output,rng,files=files)
            else:prepare_group(source,output,rng)
    else:
        if not a.variant:raise ValueError('--variant required')
        train(a.prepared,a.output,a.variant,f'cuda:{a.device}',a.epochs,a.steps)


if __name__=='__main__':main()
