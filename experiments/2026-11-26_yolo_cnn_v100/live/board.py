"""Display frame-aligned CNN and YOLO outputs and relay manual Gazebo controls."""

import argparse
import base64
import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys
from threading import Lock, Thread
from time import monotonic, sleep
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import cv2
import numpy as np
import torch
from ultralytics import YOLO

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from candidate import CandidateCNN
from evaluate import infer_cnn, infer_yolo


class Board:
    def __init__(self, token):
        self.token = token
        self.lock = Lock()
        self.frame = None
        self.error = ""
        self.updated = 0
        self.cnn = CandidateCNN()
        weights = HERE.parent / "artifacts" / "weights"
        self.cnn.load_state_dict(torch.load(weights / "cnn-augmented.pth",
                                            map_location="cpu", weights_only=True))
        self.cnn.eval()
        self.yolo = YOLO(weights / "yolo-augmented.pt").to("cpu")

    def bridge(self, path, body=None):
        request = Request("http://192.168.252.2:8793" + path, data=body,
                          headers={"X-Sim-Token": self.token,
                                   "Content-Type": "application/json"})
        return urlopen(request, timeout=1.5)

    def score(self):
        previous = None
        while True:
            try:
                with self.bridge("/frame") as incoming:
                    if incoming.status == 204:
                        sleep(.09)
                        continue
                    sequence = int(incoming.headers["X-Frame"])
                    if previous == sequence:
                        sleep(.06)
                        continue
                    stamp = float(incoming.headers["X-Sim-Time"])
                    jpeg = incoming.read()
                image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError("Could not decode simulation camera frame")
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                start = monotonic()
                cnn = infer_cnn(self.cnn, gray, "cpu", score_floor=0)[:5]
                cnn_ms = (monotonic() - start) * 1000
                start = monotonic()
                yolo = sorted(infer_yolo(self.yolo, image),
                              key=lambda hit: hit["score"], reverse=True)[:5]
                yolo_ms = (monotonic() - start) * 1000
                payload = dict(sequence=sequence, sim_time=stamp, width=image.shape[1],
                               height=image.shape[0], jpeg=base64.b64encode(jpeg).decode(),
                               cnn=cnn, yolo=yolo, cnn_ms=cnn_ms, yolo_ms=yolo_ms)
                with self.lock:
                    self.frame = payload
                    self.updated = monotonic()
                    self.error = ""
                previous = sequence
            except (OSError, ValueError, KeyError, RuntimeError) as error:
                with self.lock:
                    self.error = str(error)
                sleep(.3)


def handler(board, port):
    origin = {f"http://127.0.0.1:{port}", f"http://localhost:{port}"}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, status, body=b"", content="application/json"):
            self.send_response(status)
            self.send_header("Content-Type", content)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            if self.path == "/":
                return self.send(200, (HERE / "index.html").read_bytes(),
                                 "text/html; charset=utf-8")
            if self.path == "/api/frame":
                with board.lock:
                    data = dict(frame=board.frame, error=board.error,
                                age=monotonic() - board.updated if board.frame else None)
                return self.send(200, json.dumps(data).encode())
            if self.path == "/api/state":
                try:
                    with board.bridge("/state") as state:
                        return self.send(200, state.read())
                except OSError:
                    return self.send(503)
            return self.send(404)

        def do_POST(self):
            if self.path != "/api/drive" or self.headers.get("Origin") not in origin:
                return self.send(403)
            try:
                length = int(self.headers.get("Content-Length", 0))
                if not 1 <= length <= 256 or self.headers.get_content_type() != "application/json":
                    raise ValueError
                data = json.loads(self.rfile.read(length))
                linear, angular = float(data["linear"]), float(data["angular"])
                if not math.isfinite(linear) or not math.isfinite(angular):
                    raise ValueError
                if abs(linear) > .24 or abs(angular) > 1.0:
                    raise ValueError
            except (ValueError, TypeError, KeyError):
                return self.send(400)
            try:
                body = json.dumps(dict(linear=linear, angular=angular)).encode()
                with board.bridge("/drive", body) as response:
                    return self.send(response.status)
            except HTTPError as error:
                return self.send(error.code)
            except URLError:
                return self.send(503)

    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--token-file", type=Path, required=True)
    parser.add_argument("--port", type=int, default=8792)
    args = parser.parse_args()
    torch.set_num_threads(4)
    cv2.setNumThreads(2)
    board = Board(args.token_file.read_text().strip())
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler(board, args.port))
    Thread(target=board.score, daemon=True).start()
    print(f"Gazebo marker board http://127.0.0.1:{args.port}/", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
