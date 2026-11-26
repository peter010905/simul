"""Mine high-scoring, annotation-safe false proposals on training frames."""
import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import torch

from train_cnn import read_boxes,intersection


def main():
    p=argparse.ArgumentParser();p.add_argument('--prepared',type=Path,required=True)
    p.add_argument('--weights',type=Path,required=True);p.add_argument('--candidate',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--variant',choices=['plain','augmented'],required=True)
    p.add_argument('--device',type=int,required=True);p.add_argument('--frames',type=int,default=3000)
    a=p.parse_args();a.output.parent.mkdir(parents=True,exist_ok=True)
    sys.path.insert(0,str(a.candidate));from candidate import CandidateCNN,proposals,crop,input_bounds,box_iou
    cv2.setNumThreads(2);torch.set_num_threads(2)
    torch_device=f'cuda:{a.device}'
    model=CandidateCNN().to(torch_device)
    model.load_state_dict(torch.load(a.weights,map_location=torch_device,weights_only=True));model.eval()
    rng=np.random.default_rng(61264)
    source=a.prepared/'plain/train'
    files=sorted((source/'images').glob('*.jpg'))
    take=sorted(rng.choice(len(files),min(a.frames,len(files)),replace=False))
    patches=[];records=[];counts=dict(frames=0,contours=0,eligible=0,mined=0)
    for n,index in enumerate(take):
        file=files[index];gray=cv2.imread(str(file),cv2.IMREAD_GRAYSCALE)
        gt=read_boxes(source/'labels'/file.with_suffix('.txt').name)
        proposals_here,_=proposals(gray);counts['frames']+=1;counts['contours']+=len(proposals_here)
        eligible=[]
        for region in proposals_here:
            x0,y0,x1,y1=input_bounds(region.box)
            patch_box=(x0,y0,x1-x0,y1-y0)
            if any(intersection(patch_box,(x-4,y-4,w+8,h+8))>0 for x,y,w,h in gt):continue
            eligible.append(region)
        counts['eligible']+=len(eligible)
        found=[]
        with torch.inference_mode():
            for first in range(0,len(eligible),256):
                sub=eligible[first:first+256]
                if not sub:continue
                data=np.stack([crop(gray,r.box) for r in sub])
                x=torch.from_numpy(data[:,None].copy()).to(torch_device,dtype=torch.float32)/255
                scores=torch.sigmoid(model(x)).cpu().numpy()
                found.extend((float(score),r,patch) for score,r,patch in zip(scores,sub,data) if score>=.5)
        kept=[]
        for score,region,patch in sorted(found,key=lambda x:x[0],reverse=True):
            if any(box_iou(region.box,old[1].box)>.3 for old in kept):continue
            kept.append((score,region,patch))
            if len(kept)==4:break
        for score,region,patch in kept:
            patches.append(patch);records.append(dict(file=file.name,box=region.box,score=score))
        counts['mined']+=len(kept)
        if n%300==0:print(a.variant,n+1,'/',len(take),'mined',counts['mined'],flush=True)
    np.savez_compressed(a.output,images=np.stack(patches))
    a.output.with_suffix('.json').write_text(json.dumps(dict(variant=a.variant,counts=counts,records=records),indent=2))
    print(counts,flush=True)


if __name__=='__main__':main()
