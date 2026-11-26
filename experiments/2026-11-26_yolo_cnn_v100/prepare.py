"""Make reproducible full-frame YOLO and contour-CNN training sets.

The source is a pinned public COCO dataset. Validation images 1..500 are for
model selection; images 501..1500 stay sealed until final evaluation.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path

import cv2
import numpy as np


def coco(data, split):
    obj = json.loads((data / 'annotations' / f'instances_{split}2017.json').read_text())
    ann = defaultdict(list)
    for row in obj['annotations']:
        ann[row['image_id']].append(row)
    return obj['images'], ann


def labels(rows, width=640, height=480):
    result = []
    for row in rows:
        x, y, w, h = row['bbox']
        x1, y1 = max(0., x), max(0., y)
        x2, y2 = min(width, x+w), min(height, y+h)
        if x2-x1 >= 2 and y2-y1 >= 2:
            result.append(f'0 {(x1+x2)/(2*width):.7f} {(y1+y2)/(2*height):.7f} {(x2-x1)/width:.7f} {(y2-y1)/height:.7f}')
    return result


def transform(image, rows, rng):
    h, w = image.shape[:2]
    # A camera-like homography affects the scene and every annotated marker.
    # No unlabeled marker is pasted into a nominally negative frame.
    angle = float(rng.uniform(-28, 28))
    scale = float(rng.uniform(.90, 1.08))
    affine = cv2.getRotationMatrix2D((w/2, h/2), angle, scale)
    rotation = np.vstack([affine, [0., 0., 1.]]).astype(np.float32)
    src = np.float32([[0, 0], [w-1, 0], [w-1, h-1], [0, h-1]])
    jitter = np.float32(rng.uniform(-.055, .055, (4, 2)) * [w, h])
    perspective = cv2.getPerspectiveTransform(src, src+jitter)
    matrix = perspective @ rotation
    output = cv2.warpPerspective(image, matrix, (w, h), borderMode=cv2.BORDER_REPLICATE)
    if rng.random() < .65:
        sigma = float(rng.uniform(.45, 1.55))
        output = cv2.GaussianBlur(output, (0, 0), sigma)
    elif rng.random() < .5:
        size = int(rng.choice([3, 5]))
        kernel = np.zeros((size, size), np.float32)
        kernel[size//2] = 1/size
        rotation_angle = float(rng.uniform(0, 180))
        kernel = cv2.warpAffine(kernel, cv2.getRotationMatrix2D((size/2, size/2), rotation_angle, 1), (size, size))
        kernel /= max(kernel.sum(), 1e-8)
        output = cv2.filter2D(output, -1, kernel)
    converted = []
    for row in rows:
        x, y, bw, bh = row['bbox']
        corners = np.float32([[x,y], [x+bw,y], [x+bw,y+bh], [x,y+bh]]).reshape(-1,1,2)
        points = cv2.perspectiveTransform(corners, matrix).reshape(4,2)
        low = np.maximum(points.min(0), 0)
        high = np.minimum(points.max(0), [w,h])
        if np.all(high-low >= 3):
            converted.append({'bbox':[float(low[0]),float(low[1]),float(high[0]-low[0]),float(high[1]-low[1])]})
    return output, converted


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--data',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    cv2.setNumThreads(2)
    rng=np.random.default_rng(261126)
    for variant in ['plain','augmented']:
        for split in ['train','val','test']:
            for kind in ['images','labels']:
                (a.output/variant/split/kind).mkdir(parents=True,exist_ok=True)
    frames={};patches={}
    for split in ['train','val']:
        items,annotations=coco(a.data,split)
        for i,item in enumerate(items):
            dest='train' if split=='train' else 'val' if i<500 else 'test'
            source=(a.data/f'{split}2017'/item['file_name']).resolve()
            assert source.is_file(),source
            name=f'{split}_{item["file_name"]}'
            rows=annotations[item['id']]
            for variant in ['plain','augmented']:
                imgdir=a.output/variant/dest/'images'
                labeldir=a.output/variant/dest/'labels'
                link=imgdir/name
                if link.is_symlink() and not link.exists():link.unlink()
                if not link.exists():link.symlink_to(source)
                (labeldir/Path(name).with_suffix('.txt')).write_text('\n'.join(labels(rows))+'\n' if rows else '')
            if dest=='train' and i%2==0:
                image=cv2.imread(str(source),cv2.IMREAD_COLOR)
                augmented,new_rows=transform(image,rows,rng)
                aname=f'aug_{name}'
                cv2.imwrite(str(a.output/'augmented/train/images'/aname),augmented,[cv2.IMWRITE_JPEG_QUALITY,92])
                (a.output/'augmented/train/labels'/Path(aname).with_suffix('.txt')).write_text('\n'.join(labels(new_rows))+'\n' if new_rows else '')
            if i%1000==0:print('frames',split,i+1,'/',len(items),flush=True)
    for variant in ['plain','augmented']:
        path=a.output/variant
        (path/'data.yaml').write_text(f'path: {path.resolve()}\ntrain: train/images\nval: val/images\ntest: test/images\nnames:\n  0: aruco_marker\n')
    (a.output/'manifest.json').write_text(json.dumps({'source_revision':'7506a466db549c3ca1e187299316196acca2819c','train_frames':10000,'calibration_frames':500,'sealed_test_frames':1000,'additional_augmented_train_frames':5000,'frame_size':[640,480],'augmentation':['rotation','perspective','gaussian_or_motion_blur'],'seed':261126,'id_task':False},indent=2))


if __name__=='__main__':main()
