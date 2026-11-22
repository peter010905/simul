"""Anonymous, image-only proposals measured on native pixels. No ID is registered here."""
import cv2
import numpy as np

# A 10x10 patch holds an outer white margin, a dark rim, and a textured centre.
QUAD = np.float32([[2,2],[9,2],[9,9],[2,9]])
Y, X = np.indices((12,12))
R = np.maximum(abs(X-5.5),abs(Y-5.5))
MARGIN = R >= 4.5
RIM = R == 3.5
CORE = R <= 2.5


def candidates(gray, decoded, rejected):
    hits = []
    height, width = gray.shape
    for named, quads in ((True, decoded), (False, rejected)):
        for quad in quads:
            q = np.asarray(quad, dtype=np.float32).reshape(4,2)
            length = np.linalg.norm(q - np.roll(q,1,axis=0), axis=1)
            short = float(length.min())
            if not 6. <= short < 10. or short < .52 * max(length):
                continue
            if np.any(q < 3) or np.any(q > [width-4,height-4]):
                continue
            patch = cv2.warpPerspective(gray, cv2.getPerspectiveTransform(q,QUAD),(12,12))
            white, black = float(np.median(patch[MARGIN])), float(np.median(patch[RIM]))
            contrast = white-black
            if contrast < 35:
                continue
            threshold = black+.48*contrast
            if np.mean(patch[MARGIN]>threshold)<.72 or np.mean(patch[RIM]<=threshold)<.62:
                continue
            bits = patch[3:9,3:9]>threshold
            if not .08<bits.mean()<.84 or min(np.count_nonzero(np.diff(bits,axis=a)) for a in (0,1))<2:
                continue
            u,v=map(float,q.mean(axis=0))
            if any(abs(u-h[0])<4 and abs(v-h[1])<4 for h in hits):
                continue
            hits.append((u,v,short,named))
    return sorted(hits,key=lambda h:(h[3],h[2]),reverse=True)[:8]
