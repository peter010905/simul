"""Explicit, same-origin board commands for the prepared hardware session."""
import hmac
import ipaddress
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import urlsplit


# The first two actions remain available to existing clients.
START_WORKSPACES = {
    "start": None,
    "start-r32": "r32_drive_original18_ws",
    "start-v3-mppi": "face_intent_v3_sweep_ws",
    "start-mixed": "flow_v3_r32_print_ws",
    "start-upstream-r32": "r32_upstream_5e95333_ws",
}
LABELS = {"start": "Flow v4", "start-r32": "R32 주행 혼합 (이전)",
          "start-v3-mppi": "기존 · Flow v3", "start-mixed": "Mixed · Flow v3 + R32",
          "start-upstream-r32": "R32 Resume · 5e95333"}


def valid_request(headers, token):
    """No GET action, cross-origin form command or DNS-rebinding hostname."""
    host, origin = headers.get("Host", ""), headers.get("Origin", "")
    try:
        parsed = urlsplit(origin)
        name = urlsplit("http://" + host).hostname
        if name != "localhost":
            ipaddress.ip_address(name)
    except (ValueError, TypeError):
        return False
    return (parsed.scheme in ("http", "https") and parsed.netloc == host
            and headers.get("Content-Type", "").split(";", 1)[0] == "application/json"
            and hmac.compare_digest(headers.get("X-OSP-Control-Token", ""), token))


def tail(path, size=6000):
    try:
        with path.open("rb") as stream:
            stream.seek(0, 2)
            stream.seek(max(0, stream.tell() - size))
            return stream.read().decode("utf-8", errors="replace")
    except FileNotFoundError:
        return ""


def supervisor_alive(state):
    try:
        if state["boot_id"] != Path("/proc/sys/kernel/random/boot_id").read_text().strip():
            return False
        fields = Path(f'/proc/{state["supervisor_pid"]}/stat').read_text().rsplit(")", 1)[1].split()
        return fields[0] != "Z" and fields[19] == state["supervisor_start"]
    except (OSError, KeyError, IndexError):
        return False


class BoardControl:
    def __init__(self, repo, workspace, domain, ntp_server="192.168.185.3", duration=600., state_dir=None):
        self.repo, self.workspace = Path(repo), Path(workspace)
        self.domain, self.ntp_server, self.duration = domain, ntp_server, duration
        self.state_dir = Path(state_dir) if state_dir else Path.home() / ".local/state/osp-robot" / ("domain-" + str(domain))
        self.state_dir.mkdir(parents=True, exist_ok=True)
        self.token = secrets.token_urlsafe(32)
        self.lock = threading.RLock()
        self.start_process = self.stop_process = None
        self.start_log = self.stop_log = None
        self.started_at = 0.
        self.selected = None

    def read_state(self):
        try:
            return json.loads((self.state_dir / "session.json").read_text())
        except (OSError, ValueError):
            return {}

    def command(self, action):
        workspace = self.workspace
        native = action == "start-upstream-r32"
        if START_WORKSPACES.get(action):
            workspace = self.workspace.parent / START_WORKSPACES[action]
            action = "start"
        command = [sys.executable, "-u", str(self.repo / "scripts/robot_session.py"), action,
                   "--repo", str(self.repo), "--workspace", str(workspace),
                   "--domain", str(self.domain), "--ntp-server", self.ntp_server,
                   "--duration", str(self.duration)]
        if native:
            return ["bash", "-c", 'source /opt/ros/jazzy/setup.bash && source "$1/install/setup.bash" && shift && exec "$@"',
                    "r32-board", str(workspace), *command, "--runtime", "upstream-r32"]
        if action == "start":
            command = ["bash", str(self.repo / "run_policy.sh"), "--mode", "robot",
                       "--workspace", str(workspace), "--policy",
                       str(self.repo / "policy_releases/2026-11-13_face_intent_v2/policy.json"),
                       "--", *command]
        return command

    def spawn(self, action):
        path = self.state_dir / ("board-" + action + "-" + secrets.token_hex(6) + ".log")
        env = dict(os.environ, ROS_DOMAIN_ID=str(self.domain), OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
        with path.open("wb") as output:
            process = subprocess.Popen(self.command(action), cwd=self.repo, env=env,
                                       stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        return process, path

    def snapshot(self):
        with self.lock:
            state = self.read_state()
            alive = supervisor_alive(state)
            start_alive = self.start_process is not None and self.start_process.poll() is None
            stop_alive = self.stop_process is not None and self.stop_process.poll() is None
            belongs = state.get("requested_wall_epoch_s", 0.) >= self.started_at
            phase, detail = "idle", "구동 버튼을 누르면 시간·센서를 확인하고 출발합니다."
            if stop_alive:
                phase, detail = "stopping", "주행 작업 취소 및 실제 정지 확인 중"
            elif alive:
                phase = state.get("phase", "starting")
                detail = {"preflight": "시간·센서 상태 확인 중", "starting": "지도·카메라·주행 준비 중",
                          "running": "주행 중", "stopping": "주행 종료 중"}.get(phase, phase)
            elif self.stop_process is not None and self.stop_process.poll() is not None:
                if self.stop_process.returncode == 0:
                    phase, detail = "stopped", "주행 종료 · 실제 정지 확인 완료"
                else:
                    phase, detail = "stop_unverified", "정지 요청을 보냈지만 실제 속도를 확인하지 못했습니다."
            elif belongs and state.get("phase") == "stop_unverified":
                phase, detail = "stop_unverified", "주행 프로세스가 종료됐지만 실제 속도를 확인하지 못했습니다."
            elif belongs and state.get("phase") == "not_started":
                phase, detail = "failed", "마지막 요청에서 주행을 시작하지 못했습니다."
            elif belongs and state.get("phase") == "stopped":
                phase, detail = "stopped", "주행 종료 · 실제 정지 확인 완료" if state.get("stationary_verified") else "주행 프로세스 종료"
            elif start_alive:
                phase, detail = "queued", "구동 요청 접수 · 실행 준비 중"
            elif self.start_process is not None and self.start_process.returncode:
                phase, detail = "failed", "구동 실패"
            log_path = self.stop_log if self.stop_process is not None else self.start_log
            log = tail(log_path) if log_path else ""
            errors = [line for line in log.splitlines() if line.startswith(("RuntimeError:", "TimeoutError:", "FileNotFoundError:"))]
            if errors and phase == "failed":
                error = errors[-1]
                translations = {"waiting_for_scan": "LiDAR 데이터를 받지 못했습니다. 로봇 센서 연결을 확인하세요.",
                                "waiting_for_camera": "카메라 데이터를 받지 못했습니다.",
                                "robot_docked": "로봇이 도크에 있습니다. 도크 밖으로 이동한 뒤 구동하세요.",
                                "active_hazard_types": "범퍼·낭떠러지·바퀴 상태를 확인하세요.",
                                "clock": "시간 동기화를 확인하지 못해 출발하지 않았습니다."}
                detail = next((text for key, text in translations.items() if key in error), error)
            busy = phase in ("queued", "preflight", "starting", "running", "stopping")
            events = []
            if state.get("output") and (alive or belongs):
                for line in tail(Path(state["output"]) / "startup.jsonl").splitlines():
                    try:
                        events.append(json.loads(line))
                    except ValueError:
                        continue
            selected = self.selected
            if belongs and state.get("workspace"):
                name = Path(state["workspace"]).name
                selected = next((key for key, value in START_WORKSPACES.items() if value == name), "start")
            return dict(policy=LABELS.get(selected), enabled=True, token=self.token, phase=phase, detail=detail,
                        can_start=not busy, can_stop=True, events=events[-8:])

    def request(self, action):
        with self.lock:
            if action in START_WORKSPACES:
                if not self.snapshot()["can_start"]:
                    raise RuntimeError("이미 구동 준비 또는 주행 중입니다.")
                self.stop_process = None
                self.stop_log = None
                self.started_at = time.time()
                self.start_process, self.start_log = self.spawn(action)
                self.selected = action
            elif action == "stop":
                if self.stop_process is not None and self.stop_process.poll() is None:
                    return self.snapshot()
                # Cancel a queued wrapper too, before it can create its supervisor.
                if self.start_process is not None and self.start_process.poll() is None:
                    try:
                        os.killpg(self.start_process.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                self.stop_process, self.stop_log = self.spawn("stop")
            else:
                raise ValueError("Unknown control action")
            return self.snapshot()
