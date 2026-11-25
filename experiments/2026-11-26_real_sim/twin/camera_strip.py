"""Sparse timestamped thumbnails from the recorded robot camera, kept on the private VM."""

import argparse
import json
from pathlib import Path
import sys
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from persistent_scene import stream, stamp


def extract(logs, library, out):
    for ident, info in json.loads(library.read_text()).items():
        run = logs / ident
        start = next(
            json.loads(s)["wall_epoch_s"]
            for s in (run / "artifacts/startup.jsonl").read_text().splitlines()
            if json.loads(s)["event"] == "mission_started"
        )
        folder = out / ident
        folder.mkdir(exist_ok=True, parents=True)
        last = -10.0
        index = []
        for _, msg in stream(run, ["/oakd/rgb/preview/image_raw/compressed"]):
            t = stamp(msg.header.stamp) - start
            if t < 0 or t - last < 0.95 or (info["duration"] and t > info["duration"]):
                continue
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            name = f"{len(index):05d}.jpg"
            if not cv2.imwrite(
                str(folder / name), frame, [cv2.IMWRITE_JPEG_QUALITY, 78]
            ):
                raise OSError(name)
            last = t
            index.append([round(t, 3), name])
        (folder / "index.json").write_text(json.dumps(index))
        print(ident, len(index))


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for k in ("logs", "library", "out"):
        p.add_argument(k, type=Path)
    extract(**vars(p.parse_args()))
