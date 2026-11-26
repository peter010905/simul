"""Offline opt-in visual candidates before legacy square/bit-structure gates.

No ID decoding, map access, ROS publishing or robot control. Proposals are image
regions, not claims that a marker or a traversable viewpoint exists.
"""
from dataclasses import dataclass
import math

import cv2
import numpy as np
import torch
from torch import nn


@dataclass(frozen=True)
class Proposal:
    box: tuple
    source: str

    @property
    def center(self):
        x,y,w,h=self.box
        return x+w/2,y+h/2


def box_iou(a,b):
    x,y,w,h=a;u,v,s,t=b
    inter=max(0,min(x+w,u+s)-max(x,u))*max(0,min(y+h,v+t)-max(y,v))
    return inter/max(1e-9,w*h+s*t-inter)


def quad_box(q):
    q=np.float32(q).reshape(-1,2)
    x,y=q.min(0);u,v=q.max(0)
    return float(x),float(y),float(u-x),float(v-y)


def proposals(gray):
    """No aspect-ratio, four-corner, white-rim, payload-bit or ID requirement.

    The remaining gates are a minimum measurable region and a maximum region
    area. Both counts are audited, not asserted to be perfect recall.
    """
    if gray.ndim!=2 or gray.dtype!=np.uint8:raise ValueError('native mono8 image required')
    h,w=gray.shape
    masks=[cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV|cv2.THRESH_OTSU)[1],
           cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,15,5)]
    result=[];counts=dict(contours=0,too_small=0,too_large=0,duplicates=0)
    seen=set()
    for source,mask in zip(['global','local'],masks):
        for contour in cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[0]:
            counts['contours']+=1
            x,y,bw,bh=cv2.boundingRect(contour)
            if min(bw,bh)<3 or bw*bh<16:
                counts['too_small']+=1;continue
            if bw*bh>gray.size*.4:
                counts['too_large']+=1;continue
            key=(x//2,y//2,(x+bw)//2,(y+bh)//2)
            if key in seen:
                counts['duplicates']+=1;continue
            seen.add(key);result.append(Proposal((x,y,bw,bh),source))
    counts['retained']=len(result)
    return result,counts


def input_bounds(box):
    x,y,w,h=box;cx,cy=x+w/2,y+h/2
    bw,bh=max(5,math.ceil(w*1.32)),max(5,math.ceil(h*1.32))
    x0,y0=math.floor(cx-bw/2),math.floor(cy-bh/2)
    return x0,y0,x0+bw,y0+bh


def crop(gray,box):
    """Local rectangular context, then aspect-preserving letterbox to 48x48.

    A long thin contour must not acquire an unrelated nearby marker merely
    because its context was expanded to a large square. No rectification or
    four-corner dependency is used. Constant letterbox pixels add no evidence.
    """
    x0,y0,x1,y1=input_bounds(box)
    left,top,right,bottom=max(0,-x0),max(0,-y0),max(0,x1-gray.shape[1]),max(0,y1-gray.shape[0])
    region=gray[max(0,y0):min(gray.shape[0],y1),max(0,x0):min(gray.shape[1],x1)]
    if not region.size:raise ValueError('proposal outside image')
    region=cv2.copyMakeBorder(region,top,bottom,left,right,cv2.BORDER_REPLICATE)
    scale=min(48/region.shape[0],48/region.shape[1]);size=(max(1,round(region.shape[1]*scale)),max(1,round(region.shape[0]*scale)))
    resized=cv2.resize(region,size,interpolation=cv2.INTER_AREA if scale<1 else cv2.INTER_LINEAR)
    output=np.full((48,48),128,np.uint8);ox=(48-size[0])//2;oy=(48-size[1])//2
    output[oy:oy+size[1],ox:ox+size[0]]=resized
    return output


class CandidateCNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.features=nn.Sequential(nn.Conv2d(1,12,3,padding=1),nn.ReLU(),nn.MaxPool2d(2),
            nn.Conv2d(12,24,3,padding=1),nn.ReLU(),nn.MaxPool2d(2),
            nn.Conv2d(24,32,3,padding=1),nn.ReLU(),nn.MaxPool2d(2))
        self.classifier=nn.Sequential(nn.Flatten(),nn.Linear(32*6*6,48),nn.ReLU(),nn.Dropout(.15),nn.Linear(48,1))

    def forward(self,x):return self.classifier(self.features(x)).squeeze(1)


def scores(model,gray,regions,batch=128):
    if not regions:return []
    patches=np.stack([crop(gray,r.box) for r in regions])
    answer=[]
    with torch.inference_mode():
        for i in range(0,len(patches),batch):
            x=torch.from_numpy(patches[i:i+batch,None].copy()).float()/255
            answer.extend(torch.sigmoid(model(x)).tolist())
    return answer


def select(regions,probabilities,threshold):
    selected=[]
    for index in sorted(range(len(regions)),key=lambda i:probabilities[i],reverse=True):
        if probabilities[index]<threshold:break
        r=regions[index]
        if any(box_iou(r.box,old['box'])>.3 for old in selected):continue
        x,y,w,h=r.box
        selected.append(dict(box=list(r.box),uv=list(r.center),score=probabilities[index],
                             source='geometry_free_cnn',id=None,
                             native_box_short_edge=min(w,h),observation_only=True))
    return selected


def legacy_paper_reason(gray,quad):
    """Mirror hybrid_visual.paper_candidate solely to audit first rejection.

    A separate equivalence check compares this result with the frozen function.
    This audit is never an extra gate in the CNN path.
    """
    from osp_mission.hybrid_visual import DEST,RIM,BORDER,INNER
    q=np.float32(quad).reshape(4,2);edges=np.linalg.norm(q-np.roll(q,1,axis=0),axis=1)
    if edges.min()<10:return 'min_short_edge_10px'
    if edges.min()<.46*edges.max():return 'edge_ratio_0.46'
    if np.any(q<4) or np.any(q>[gray.shape[1]-5,gray.shape[0]-5]):return 'image_margin'
    patch=cv2.warpPerspective(gray,cv2.getPerspectiveTransform(q,DEST),(56,56))
    white,black=np.median(patch[RIM]),np.median(patch[BORDER])
    if white-black<38:return 'contrast_38'
    bits=patch>(white+black)/2
    if bits[RIM].mean()<.78:return 'white_rim_0.78'
    if bits[BORDER].mean()>.15:return 'dark_border_0.15'
    if not .15<bits[INNER].mean()<.82:return 'payload_fraction'
    sample=bits[13:43:4,13:43:4]
    if np.count_nonzero(np.diff(sample,axis=0))<4 or np.count_nonzero(np.diff(sample,axis=1))<4:return 'two_axis_transitions'
    return 'accepted'
