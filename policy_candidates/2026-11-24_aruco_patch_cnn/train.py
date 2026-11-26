"""Train a crop CNN on photographed confusers and synthetic Original markers."""
import argparse
import csv
import json
from pathlib import Path
import time

import cv2
import numpy as np
import torch
from torch import nn

from patch_cnn import MarkerPatchCNN, crop, normalize


DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
SHAPE = np.float32([[0, 0], [71, 0], [71, 71], [0, 71]])
CORNERS = np.float32([[-1, -1], [1, -1], [1, 1], [-1, 1]])


def synthetic(backgrounds, ids, count, rng):
    images, labels = [], []
    for i in range(count):
        positive = i % 2 == 0
        background = backgrounds[int(rng.integers(len(backgrounds)))]
        y = int(rng.integers(background.shape[0] - 64))
        x = int(rng.integers(background.shape[1] - 64))
        image = background[y:y + 64, x:x + 64].copy()
        short = rng.uniform(5., 21.)
        long = min(28., short / rng.uniform(.38, 1.))
        w, h = (long, short) if rng.random() < .5 else (short, long)
        a = rng.uniform(-np.pi, np.pi)
        rotation = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
        quad = ((CORNERS * [w / 2, h / 2]) @ rotation.T
                + rng.uniform(-.14, .14, (4, 2)) * short + rng.uniform(27., 37., (1, 2))).astype(np.float32)
        if positive:
            ident = int(rng.choice(ids))
            pattern = cv2.aruco.generateImageMarker(DICTIONARY, ident, 7)
        else:
            pattern = np.zeros((7, 7), dtype=np.uint8)
            pattern[1:-1, 1:-1] = rng.integers(0, 2, (5, 5)) * 255
            if rng.random() < .30:
                pattern[1:-1, 1:-1] = 0
                r, c = rng.integers(1, 5, size=2)
                pattern[r:r + 2, c:c + 2] = 255
        dark, light = rng.integers(5, 100), rng.integers(150, 255)
        paper = np.pad(np.where(pattern > 0, light, dark).astype(np.uint8), 1, constant_values=light)
        paper = cv2.resize(paper, (72, 72), interpolation=cv2.INTER_NEAREST)
        # Warp at twice the image size before sampling at the native resolution.
        surround = quad.mean(axis=0) + (quad-quad.mean(axis=0))*9/7
        transform = cv2.getPerspectiveTransform(SHAPE, 2*surround)
        canvas = cv2.resize(image, (128, 128), interpolation=cv2.INTER_LINEAR)
        cv2.warpPerspective(paper, transform, (128, 128), dst=canvas,
                            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_TRANSPARENT)
        image = cv2.resize(canvas, (64, 64), interpolation=cv2.INTER_AREA)
        if rng.random() < .5:
            image = cv2.GaussianBlur(image, (5, 5), rng.uniform(.3, .9))
        _, compressed = cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,int(rng.integers(40,95))])
        image = cv2.imdecode(compressed,cv2.IMREAD_GRAYSCALE)
        proposal = quad + rng.normal(0, min(.85, .075*short), quad.shape).astype(np.float32)
        if positive and rng.random() < .20:
            # A detector can supply only the visible portion of a print.
            side = int(rng.integers(4))
            proposal[side] += .16 * (proposal[(side+2)%4] - proposal[side])
            following = (side+1)%4
            proposal[following] += .16 * (proposal[(following+2)%4] - proposal[following])
        images.append(crop(image,proposal))
        labels.append(float(positive))
    return np.asarray(images), np.asarray(labels, np.float32)


def augment(patch, rng):
    sample = np.rot90(patch, int(rng.integers(4)))
    gain, bias = rng.uniform(.8, 1.2), rng.uniform(-22, 22)
    return np.uint8(np.clip(sample * gain + bias, 0, 255))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('gallery', type=Path)
    parser.add_argument('review', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    torch.manual_seed(42)
    torch.set_num_threads(2)
    cv2.setNumThreads(1)
    rng = np.random.default_rng(42)
    doc = (args.gallery/'gallery-data.js').read_text()
    data = json.loads(doc.removeprefix('window.candidateData=').strip().removesuffix(';'))
    review = json.loads(args.review.read_text())
    train_rows = data['250px']['rows']
    by_index = {r['index']:r for r in train_rows}
    train_neg = [[r for r in train_rows if r['group']==by_index[i]['group'] and r['quad'] is not None]
                 for i in review['train_negative_indices']]
    negatives = []
    backgrounds = []
    for group in train_neg:
        for row in group[::max(1,len(group)//5)][:5]:
            image = cv2.imread(str(args.gallery/row['frame_file']),0)
            negatives.append(crop(image,row['quad']))
            backgrounds.append(image)
    positives = []
    detector = cv2.aruco.ArucoDetector(DICTIONARY)
    for row in train_rows:
        # Keep ID 0 and ID 42 out of the training data for their held-out runs.
        if row['logged_id'] != 7:
            continue
        image = cv2.imread(str(args.gallery/row['frame_file']),0)
        corners, ids, _ = detector.detectMarkers(image)
        if ids is not None:
            positives.extend(crop(image,q) for i,q in zip(ids.ravel(),corners) if i==7)
    choices = np.array([i for i in range(len(DICTIONARY.bytesList)) if i not in (0,42)])
    choices = rng.permutation(choices)
    train_x, train_y = synthetic(backgrounds,choices[80:],4800,rng)
    val_x, val_y = synthetic(backgrounds,choices[:80],600,rng)
    extra_x, extra_y = [], []
    for label, collection, repeats in [(0,negatives,8),(1,positives,8)]:
        for image in collection:
            for _ in range(repeats):
                extra_x.append(augment(image,rng))
                extra_y.append(label)
    images = torch.from_numpy(normalize(np.concatenate([train_x,np.asarray(extra_x)])))
    targets = torch.tensor(np.concatenate([train_y,np.asarray(extra_y)]),dtype=torch.float32)
    val = torch.from_numpy(normalize(val_x))
    model = MarkerPatchCNN()
    loss = nn.BCEWithLogitsLoss()
    optimizer = torch.optim.AdamW(model.parameters(), lr=.002)
    best, saved = -float('inf'), None
    for epoch in range(12):
        model.train()
        for sample in torch.randperm(len(images)).split(96):
            optimizer.zero_grad()
            objective = loss(model(images[sample]).flatten(), targets[sample])
            objective.backward()
            optimizer.step()
        model.eval()
        with torch.inference_mode():
            val_logits = model(val).flatten()
            score = float(-loss(val_logits,torch.from_numpy(val_y)))
        print(f'epoch {epoch+1}: synthetic validation loss {-score:.4f}',flush=True)
        if score > best:
            best, saved = score, {k:v.detach().clone() for k,v in model.state_dict().items()}
    model.load_state_dict(saved)
    model.eval()
    torch.save(saved,args.output/'marker-patch-cnn.pt')
    torch.jit.trace(model,torch.zeros(1,1,32,32)).save(str(args.output/'marker-patch-cnn.torchscript'))
    info = dict(training_run=data['250px']['summary']['source'], synthetic_train=len(train_x),
                separate_synthetic_id_validation=len(val_x), real_negative_groups=len(train_neg),
                real_negative_frames=len(negatives), real_positive_frames=len(positives),
                parameters=sum(p.numel() for p in model.parameters()), validation_loss=-best)
    (args.output/'training.json').write_text(json.dumps(info,indent=2)+'\n')
    print(json.dumps(info,indent=2))


if __name__ == '__main__':
    main()
