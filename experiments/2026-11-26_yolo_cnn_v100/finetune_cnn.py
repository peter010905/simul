"""Fine-tune both CNNs with matched hard-negative batches."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
import torch
from torch.nn import functional as F

from train_cnn import scores,f1


def load(path,key='images'):
    with np.load(path) as packed:return {name:packed[name] for name in packed.files}


def main():
    p=argparse.ArgumentParser();p.add_argument('--runs',type=Path,required=True);p.add_argument('--candidate',type=Path,required=True)
    p.add_argument('--variant',choices=['plain','augmented'],required=True);p.add_argument('--device',type=int,required=True)
    p.add_argument('--epochs',type=int,default=5);p.add_argument('--steps',type=int,default=500)
    a=p.parse_args();sys.path.insert(0,str(a.candidate));from candidate import CandidateCNN
    torch.set_num_threads(2);torch.manual_seed(261127);rng=np.random.default_rng(261127)
    device=f'cuda:{a.device}'
    plain=load(a.runs/'cnn-patches/plain.npz');aug=load(a.runs/'cnn-patches/augmented.npz') if a.variant=='augmented' else None
    val=load(a.runs/'cnn-patches/val.npz');hard=load(a.runs/'cnn-hard'/f'{a.variant}.npz')['images']
    sources=[plain] if aug is None else [plain,aug]
    pools=[{label:np.flatnonzero(source['labels']==label) for label in (0,1)} for source in sources]
    model=CandidateCNN().to(device)
    model.load_state_dict(torch.load(a.runs/'cnn'/a.variant/'best.pth',map_location=device,weights_only=True))
    opt=torch.optim.AdamW(model.parameters(),lr=3e-5,weight_decay=1e-4)
    out=a.runs/'cnn-hard-tuned'/a.variant;out.mkdir(parents=True,exist_ok=True)
    history=[]
    for epoch in range(a.epochs):
        model.train();losses=[]
        for _ in range(a.steps):
            source=int(rng.choice(len(sources)))
            pos=sources[source]['images'][rng.choice(pools[source][1],64,replace=True)]
            negative=sources[source]['images'][rng.choice(pools[source][0],32,replace=True)]
            hard_negative=hard[rng.choice(len(hard),32,replace=True)]
            image=np.concatenate([pos,negative,hard_negative])
            label=np.concatenate([np.ones(64,np.float32),np.zeros(64,np.float32)])
            order=rng.permutation(len(label))
            x=torch.from_numpy(image[order,None].copy()).to(device,dtype=torch.float32)/255
            y=torch.from_numpy(label[order].copy()).to(device)
            loss=F.binary_cross_entropy_with_logits(model(x),y)
            opt.zero_grad();loss.backward();opt.step();losses.append(float(loss.detach()))
        probability=scores(model,val['images'],device)
        winner=max(((f1(val['labels'],probability,t),float(t)) for t in np.arange(.05,1,.05)),key=lambda x:x[0])
        row=dict(epoch=epoch+1,mean_loss=float(np.mean(losses)),validation_patch_f1=winner[0],validation_patch_threshold=winner[1])
        history.append(row);torch.save(model.state_dict(),out/f'epoch-{epoch+1}.pth')
        (out/'history.json').write_text(json.dumps(history,indent=2));print(a.variant,row,flush=True)


if __name__=='__main__':main()
