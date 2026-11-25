"""Local simulation comparison board. Only a fixed, simulation-only launcher can be invoked."""

import argparse
from bisect import bisect_left
import csv
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import subprocess
import threading
import time
from urllib.parse import urlsplit, parse_qs

from comparison import compare, first_displacements, trial

HERE = Path(__file__).resolve().parent
SCENES = {
    "desks": "scene16-metric",
    "returns": "scene16-parity",
    "departure": "scene16-departure",
}


class Board:
    def __init__(self, root, workspace):
        self.root = root.resolve()
        self.workspace = workspace.resolve()
        self.ref = self.read("scene16-r1/reference.json")
        self.ref["first_displacements"] = first_displacements(
            self.ref["trace"], self.ref["start"][:2]
        )
        self.lock = threading.Lock()
        self.cache = {}
        self.job = None
        self.frame_indices = {}
        self.plot_lock = threading.Lock()

    def read(self, name):
        return json.loads((self.root / name).read_text())

    def folders(self):
        return sorted(
            p
            for p in self.root.glob("trial-track-*")
            if p.is_dir() and (p / "scene.json").is_file()
        )

    def result(self, p):
        key = (
            p.name,
            tuple(
                x.stat().st_mtime_ns if x.is_file() else 0
                for x in (p / "headings.csv", p / "metrics.json", p / "failure.json")
            ),
        )
        with self.lock:
            if self.cache.get(p.name, (None,))[0] != key:
                result = trial(p, self.ref)
                scene = (p / "world").resolve().parent.name
                result["scene"] = (
                    "departure"
                    if scene == "scene16-departure"
                    else "returns"
                    if scene in ("scene16-returns", "scene16-parity")
                    else "desks"
                )
                protocol = p / "provenance.json"
                proof = json.loads(protocol.read_text()) if protocol.exists() else {}
                result["policy_run"] = proof.get("policy_run", self.ref["run"])
                result["hardware_parity"] = bool(proof.get("hardware_parity", False))
                result["optical_profile"] = (
                    bool(proof.get("optical_profile", False))
                    or proof.get("workspace") == "twin43_optics_ws"
                )
                result["scanner_profile"] = bool(proof.get("scanner_profile"))
                result["time_limit_s"] = proof.get("time_limit_s", 600)
                result["paired"] = result["policy_run"] == self.ref["run"]
                if not result["paired"]:
                    result["comparison"] = {}
                logfile = self.root / (p.name + ".log")
                if (
                    not result["finished"]
                    and logfile.is_file()
                    and "Traceback (most recent call last)"
                    in logfile.read_text(errors="replace")
                ):
                    result["terminal"] = "실행 오류"
                failure = p / "failure.json"
                if not result["finished"] and failure.is_file():
                    result["terminal"] = (
                        "시뮬 종료 · 결과 미생성 (종료 코드 %s)"
                        % json.loads(failure.read_text())["observer_exit"]
                    )
                self.cache[p.name] = (key, result)
            return self.cache[p.name][1]

    def repeat(self, folder):
        def signature(run):
            path = run / "provenance.json"
            if not path.is_file():
                return None
            doc = json.loads(path.read_text())
            keys = ("workspace", "pose_run", "policy_sha256", "world_sha256")
            if not all(doc.get(k) for k in keys):
                return None
            return tuple(doc[k] for k in keys) + (
                doc.get("time_limit_s", 600),
                doc.get("source_settings_sha256"),
            )

        key = signature(folder)
        if key is None:
            return None
        peers = [p for p in self.folders() if p != folder and signature(p) == key]
        if not peers:
            return None
        peer = peers[-1]
        a = self.result(folder)
        b = self.result(peer)
        return dict(id=peer.name, comparison=compare(a["trace"], b["trace"]))

    def camera(self, kind, ident, t):
        if not math.isfinite(t) or not 0 <= t <= 610:
            return None
        if kind == "real" and ident in self.read("stages.json"):
            folder = self.root / "thumbnails" / ident
            index = folder / "index.json"
        elif kind == "sim" and any(p.name == ident for p in self.folders()):
            folder = self.root / ident / "camera"
            index = folder.parent / "frames.csv"
        else:
            return None
        if not index.is_file():
            return None
        revision = index.stat().st_mtime_ns
        with self.lock:
            cached = self.frame_indices.get(index)
            if not cached or cached[0] != revision:
                if kind == "real":
                    items = json.loads(index.read_text())
                else:
                    items = []
                    with index.open() as stream:
                        for row in csv.DictReader(stream):
                            try:
                                items.append(
                                    (
                                        float(row["elapsed_sim_s"]),
                                        f"{int(row['frame']):05d}.jpg",
                                    )
                                )
                            except (TypeError, ValueError, KeyError):
                                continue
                cached = (revision, [i[0] for i in items], [i[1] for i in items])
                self.frame_indices[index] = cached
        _, times, names = cached
        if not times:
            return None
        i = bisect_left(times, t)
        best = min(
            (j for j in (i - 1, i) if 0 <= j < len(times)),
            key=lambda j: abs(times[j] - t),
        )
        if abs(times[best] - t) > (0.65 if kind == "real" else 0.30):
            return None
        frame = folder / names[best]
        return frame.read_bytes() if frame.is_file() else None

    def decision_image(self, ident):
        folder = next((p for p in self.folders() if p.name == ident), None)
        if not folder or not self.result(folder)["paired"]:
            return None
        real = self.root / "reference-decision.npz"
        simulated = folder / "native/policy_decisions/0000.npz"
        if not real.is_file() or not simulated.is_file():
            return None
        image = folder / "first-decision.png"
        with self.plot_lock:
            if not image.exists() or image.stat().st_mtime < max(
                real.stat().st_mtime, simulated.stat().st_mtime
            ):
                from decision_plot import draw

                temp = folder / "first-decision.new.png"
                draw(real, simulated, temp)
                temp.replace(image)
            return image.read_bytes()

    def start(self, scene, policy, optical=False, scanner=False):
        if (
            scene not in SCENES
            or not (self.root / SCENES[scene] / "scene.yaml").is_file()
        ):
            raise ValueError("알 수 없는 배치")
        stages = self.read("stages.json")
        if policy not in stages:
            raise ValueError("등록된 정책을 선택해 주세요.")
        profile_key = "optical_workspace" if optical is True else "workspace"
        if scanner is True and optical is True:
            profile_key = "scanner_workspace"
        if (
            not isinstance(optical, bool)
            or not isinstance(scanner, bool)
            or (scanner and not optical)
            or profile_key not in stages[policy]
        ):
            raise ValueError("선택한 정책의 카메라 설정을 확인해 주세요.")
        stage = Path(stages[policy][profile_key])
        if not (stage / "install/setup.bash").is_file():
            raise ValueError("설치된 정책을 찾을 수 없습니다.")
        with self.lock:
            if self.job and self.job.poll() is None:
                raise ValueError("비교 주행이 이미 진행 중입니다.")
            # One native simulator per domain. The two initial calibration runs use 174 and 175.
            import psutil

            for proc in psutil.process_iter():
                try:
                    env = proc.environ()
                    if env.get("BENCH_RUN") and env.get("ROS_DOMAIN_ID") in (
                        "174",
                        "175",
                    ):
                        raise ValueError("현재 주행이 끝난 뒤 다시 실행할 수 있습니다.")
                except psutil.Error:
                    pass
            run = self.root / f"trial-track-{time.time_ns()}"
            with (self.root / (run.name + ".log")).open("w") as log:
                self.job = subprocess.Popen(
                    [
                        "bash",
                        str(HERE / "trial.sh"),
                        str(self.root / SCENES[scene]),
                        str(stage),
                        str(run),
                    ],
                    stdin=subprocess.DEVNULL,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            return run.name


class Handler(BaseHTTPRequestHandler):
    def reply(self, status, body, kind="application/json"):
        if kind == "application/json":
            body = json.dumps(body, allow_nan=False, separators=(",", ":")).encode()
        self.send_response(status)
        self.send_header(
            "Content-Type",
            kind if kind.startswith("image/") else kind + "; charset=utf-8",
        )
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        url = urlsplit(self.path)
        b = self.server.board
        if url.path in ("/", "/twin", "/twin/"):
            return self.reply(200, (HERE / "index.html").read_bytes(), "text/html")
        if url.path == "/camera.jpg":
            q = parse_qs(url.query)
            try:
                frame = b.camera(q["kind"][0], q["id"][0], float(q["time"][0]))
            except (ValueError, KeyError):
                frame = None
            if frame:
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(frame)))
                self.end_headers()
                try:
                    self.wfile.write(frame)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return
            return self.reply(404, {"error": "해당 시각 영상 없음"})
        if (
            url.path == "/registered-slam.png"
            and (b.root / "registered-slam.png").is_file()
        ):
            return self.reply(
                200, (b.root / "registered-slam.png").read_bytes(), "image/png"
            )
        if url.path == "/first-decision.png":
            ident = parse_qs(url.query).get("id", [""])[0]
            image = b.decision_image(ident)
            if image:
                return self.reply(200, image, "image/png")
            return self.reply(404, {"error": "대응하는 첫 목표 기록이 없습니다."})
        if url.path == "/api/setup":
            return self.reply(
                200,
                dict(
                    fit=b.read("scene16-metric/fit.json"),
                    tables=b.read("scene16-r1/run/simulation/demo_layout.json")[
                        "tables"
                    ],
                    returns=b.read("scene16-returns/returns.json"),
                    ref=b.ref,
                    library=b.read("library.json"),
                    hypotheses=b.read("map-hypotheses.json")
                    if (b.root / "map-hypotheses.json").is_file()
                    else {},
                    anchors=b.read("leg-anchors.json")
                    if (b.root / "leg-anchors.json").is_file()
                    else [],
                    slam_layer=b.read("registered-slam.json")
                    if (b.root / "registered-slam.json").is_file()
                    else None,
                    departure_returns=b.read("scene16-departure/returns.json")
                    if (b.root / "scene16-departure/returns.json").is_file()
                    else None,
                    policies=list(b.read("stages.json")),
                    reference_goals=b.read("reference-goals.json")
                    if (b.root / "reference-goals.json").is_file()
                    else [],
                    scanner_policies=[
                        k
                        for k, v in b.read("stages.json").items()
                        if "scanner_workspace" in v
                    ],
                    optical_policies=[
                        k
                        for k, v in b.read("stages.json").items()
                        if "optical_workspace" in v
                    ],
                    reference_decision=b.read("reference_decision.json")
                    if (b.root / "reference_decision.json").is_file()
                    else None,
                ),
            )
        if url.path == "/api/runs":
            return self.reply(
                200,
                [
                    {k: v for k, v in b.result(p).items() if k != "trace"}
                    for p in b.folders()
                ],
            )
        if url.path == "/api/run":
            name = parse_qs(url.query).get("id", [""])[0]
            for p in b.folders():
                if p.name == name:
                    return self.reply(200, dict(**b.result(p), repeat=b.repeat(p)))
        self.reply(404, {"error": "찾을 수 없습니다."})

    def do_POST(self):
        host = self.headers.get("Host", "")
        expected = f"{self.server.server_address[0]}:{self.server.server_port}"
        if (
            self.path != "/api/run"
            or host != expected
            or self.headers.get("Origin") != "http://" + expected
            or self.headers.get_content_type() != "application/json"
        ):
            return self.reply(403, {"error": "이 보드에서 실행해 주세요."})
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 256:
                raise ValueError("요청 크기가 잘못됐습니다.")
            params = json.loads(self.rfile.read(length))
            name = self.server.board.start(
                params["scene"],
                params.get("policy", self.server.board.ref["run"]),
                params.get("optical", False),
                params.get("scanner", False),
            )
        except (KeyError, TypeError, ValueError) as e:
            return self.reply(409, {"error": str(e)})
        self.reply(202, {"id": name})


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("root", type=Path)
    p.add_argument("workspace", type=Path)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8782)
    a = p.parse_args()
    server = ThreadingHTTPServer((a.host, a.port), Handler)
    server.board = Board(a.root, a.workspace)
    server.serve_forever()
