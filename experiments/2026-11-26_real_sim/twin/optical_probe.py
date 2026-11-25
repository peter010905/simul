"""Measure the rendered scene from recorded simulation images, independent of CameraInfo."""

import argparse
import csv
from pathlib import Path
import cv2
import numpy as np
from optics import fit_frames


def measure(folder):
    with (folder / "headings.csv").open() as f:
        pose = []
        for r in csv.DictReader(f):
            t = float(r["elapsed_sim_s"])
            if t > 8:
                break
            pose.append([t, float(r["yaw_rad"]), 0.0])
    frames = []
    with (folder / "frames.csv").open() as f:
        for r in csv.DictReader(f):
            t = float(r["elapsed_sim_s"])
            if t > 7.3:
                break
            if t < 0.7:
                continue
            im = cv2.imread(
                str(folder / "camera" / f"{int(r['frame']):05d}.jpg"),
                cv2.IMREAD_GRAYSCALE,
            )
            if im is not None:
                frames.append((t, im))
    return fit_frames(np.array(pose), frames, 123.67, 127.08)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("folder", type=Path)
    print(measure(p.parse_args().folder))
