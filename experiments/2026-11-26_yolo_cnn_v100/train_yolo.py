"""Train one full-frame detector with fixed settings and data split."""
import argparse
import json
from pathlib import Path
import random

import numpy as np
import torch
from ultralytics import YOLO
import ultralytics


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--prepared',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--variant',choices=['plain','augmented'],required=True)
    p.add_argument('--device',type=int,required=True)
    p.add_argument('--epochs',type=int,default=20)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    random.seed(261126);np.random.seed(261126);torch.manual_seed(261126)
    model=YOLO('yolo11n.pt')
    result=model.train(data=str((a.prepared/a.variant/'data.yaml').resolve()),project=str(a.output.resolve()),name=a.variant,
                       device=a.device,epochs=a.epochs,imgsz=640,batch=32,workers=4,seed=261126,deterministic=True,
                       patience=6,amp=True,pretrained=True,plots=False,save=True,exist_ok=True,
                       degrees=0,translate=0,scale=0,shear=0,perspective=0,flipud=0,fliplr=0,
                       mosaic=0,mixup=0,copy_paste=0,hsv_h=0,hsv_s=0,hsv_v=0,erasing=0,
                       close_mosaic=0,verbose=False)
    (a.output/a.variant/'run.json').write_text(json.dumps({'variant':a.variant,'device':a.device,'epochs_requested':a.epochs,
        'ultralytics':ultralytics.__version__,'torch':torch.__version__,'checkpoint':str((a.output/a.variant/'weights'/'best.pt').resolve()),
        'augmentation':'offline 5000 only' if a.variant=='augmented' else 'none'},indent=2))
    print(result,flush=True)


if __name__=='__main__':main()
