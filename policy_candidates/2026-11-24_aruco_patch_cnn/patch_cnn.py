"""Small grayscale classifier for ArUco Original candidate crops."""
import cv2
import numpy as np
from torch import nn


SIZE = 32
SQUARE = np.float32([[4, 4], [27, 4], [27, 27], [4, 27]])


def crop(gray, quad):
    points = np.asarray(quad, dtype=np.float32).reshape(4, 2)
    transform = cv2.getPerspectiveTransform(points, SQUARE)
    return cv2.warpPerspective(gray, transform, (SIZE, SIZE), borderMode=cv2.BORDER_REPLICATE)


def normalize(images):
    return images[:, None].astype(np.float32) * (2. / 255.) - 1.


class MarkerPatchCNN(nn.Sequential):
    def __init__(self):
        super().__init__(
            nn.Conv2d(1, 8, 3, padding=1), nn.ReLU(), nn.AvgPool2d(2),
            nn.Conv2d(8, 16, 3, padding=1), nn.ReLU(), nn.AvgPool2d(2),
            nn.Conv2d(16, 24, 3, padding=1), nn.ReLU(), nn.AdaptiveAvgPool2d((3, 3)),
            nn.Flatten(), nn.Linear(24 * 3 * 3, 1),
        )
