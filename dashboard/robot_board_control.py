"""Explicit, same-origin board commands for the prepared hardware session."""
import hmac
import ipaddress
import json
import math
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
    "prepare-focus": "v152_focus_continuity_box1122_ws",
    "prepare-sweep31": "v15231_start_sweep_640_1124_ws",
    "prepare-chair31": "v15231_chair_clutter_640_1124_ws",
    "prepare-wall-first": "v152_wall_first1123_ws",
    "prepare-balanced": "flow_v151_exposure_sync_r2_ws",
    "prepare-upstream-r32": "r32_upstream_5e95333_ws",
    "prepare-region-transit": "v15241_region_transit_r1_640_1124_ws",
    "prepare-box-safe-views": "v15240_box_safe_views_640_1124_ws",
    "prepare-unvisited-priority": "v15239_unvisited_priority_640_1124_ws",
    "prepare-marker-first-ends": "v15238_marker_first_end_walls_640_1124_ws",
    "prepare-small-return-filter": "v15237_small_return_filter_640_1124_ws",
    "prepare-box-face-recovery": "v15236_box_face_recovery_640_1124_ws",
    "prepare-auto-chair-strip": "v15235_auto_chair_strip_640_1124_ws",
    "prepare-chair-strip": "v15234_chair_strip_640_1124_ws",
    "prepare-white-legs": "v15233_white_legs_640_1124_ws",
    "prepare-glass-wall": "v15232_glass_wall_640_1124_ws",
    "start": None,
    "start-r32": "r32_drive_original18_ws",
    "start-v3-mppi": "face_intent_v3_sweep_ws",
    "start-mixed": "flow_v3_r32_full_box_ws",
    "start-thorough": "flow_v3_r32_thorough_ws",
    "start-corner": "flow_v3_r32_corner_ws",
    "start-side": "flow_v3_r32_side_ws",
    "start-general": "flow_v3_r32_general_ws",
    "start-center": "flow_v3_r32_center_ws",
    "start-wall-cues": "flow_v3_r32_wall_cues_final_ws",
    "start-cluster-exit": "flow_v3_r32_cluster_exit_ws",
    "start-marker-relook": "flow_v3_r32_marker_relook_ws",
    "start-marker-relook-045": "flow_v3_r32_marker_relook_ws",
    "start-stall-recovery": "flow_v3_r32_smooth030_ws",
    "start-crowded": "flow_v3_r32_crowded_ws",
    "start-balanced": "flow_v151_exposure_sync_r2_ws",
    "start-focus": "v152_focus_continuity_box1122_ws",
    "start-sweep31": "v15231_start_sweep_640_1124_ws",
    "start-chair31": "v15231_chair_clutter_640_1124_ws",
    "start-wall-first": "v152_wall_first1123_ws",
    "start-upstream-r32": "r32_upstream_5e95333_ws",
    "start-region-transit": "v15241_region_transit_r1_640_1124_ws",
    "start-box-safe-views": "v15240_box_safe_views_640_1124_ws",
    "start-unvisited-priority": "v15239_unvisited_priority_640_1124_ws",
    "start-marker-first-ends": "v15238_marker_first_end_walls_640_1124_ws",
    "start-small-return-filter": "v15237_small_return_filter_640_1124_ws",
    "start-box-face-recovery": "v15236_box_face_recovery_640_1124_ws",
    "start-auto-chair-strip": "v15235_auto_chair_strip_640_1124_ws",
    "start-chair-strip": "v15234_chair_strip_640_1124_ws",
    "start-white-legs": "v15233_white_legs_640_1124_ws",
    "start-glass-wall": "v15232_glass_wall_640_1124_ws",
}
LABELS = {"start": "Flow v4", "start-r32": "R32 주행 혼합 (이전)",
          "prepare-focus": "Mixed v1.5.2.1 · Focus Continuity",
          "prepare-sweep31": "Mixed v1.5.2.31 · Start Sweep",
          "prepare-chair31": "Mixed v1.5.2.31 · Chair Clutter (실험 · 범퍼 4회 기록)",
          "prepare-wall-first": "Mixed v1.5.2 · 벽 우선 복귀",
          "prepare-balanced": "Mixed v1.5.1-balanced",
          "start-v3-mppi": "기존 · Flow v3", "start-mixed": "Mixed · 상자 전면 관찰",
          "start-thorough": "Mixed · 꼼꼼 탐색",
          "start-corner": "Mixed · 각진 구조 우선",
          "start-side": "Mixed · 이동 중 옆면 관찰",
          "start-general": "Mixed · 일반 장애물 집중 탐색",
          "start-center": "Mixed · 가운데 장애물 → 벽",
          "start-wall-cues": "Mixed · 작은 벽 후보 기억 (v1.0.0)",
          "start-cluster-exit": "Mixed · 상자 재방문 줄이기 (v1.1.0)",
          "start-marker-relook": "Mixed · 판독 마커 재관찰 (v1.2 실험)",
          "start-marker-relook-045": "Mixed · 판독 마커 재관찰 (v1.2 · 0.45 m/s)",
          "start-stall-recovery": "Mixed · 부드러운 주행·복구 (v1.3.1 · 0.30 m/s)",
          "start-crowded": "Mixed · 밀집 장애물 판단 (v1.4 실험 · 0.30 m/s)",
          "start-balanced": "Mixed v1.5.1-balanced",
          "start-focus": "Mixed v1.5.2.1 · Focus Continuity",
          "start-sweep31": "Mixed v1.5.2.31 · Start Sweep",
          "start-chair31": "Mixed v1.5.2.31 · Chair Clutter (실험 · 범퍼 4회 기록)",
          "start-wall-first": "Mixed v1.5.2 · 벽 우선 복귀",
          "start-upstream-r32": "R32 Resume · 5e95333"}
LABELS["prepare-upstream-r32"] = LABELS["start-upstream-r32"]
LABELS["start-glass-wall"] = "Mixed v1.5.2.32 · Glass Wall + Marker Cooldown"
LABELS["prepare-glass-wall"] = LABELS["start-glass-wall"]
LABELS["start-white-legs"] = "Mixed v1.5.2.33 · RGB 다리 후보"
LABELS["prepare-white-legs"] = LABELS["start-white-legs"]
LABELS["start-chair-strip"] = "Mixed v1.5.2.34 · 의자 띠 제외"
LABELS["prepare-chair-strip"] = LABELS["start-chair-strip"]
LABELS["start-auto-chair-strip"] = "Mixed v1.5.2.35 · 의자 띠 자동 판정"
LABELS["prepare-auto-chair-strip"] = LABELS["start-auto-chair-strip"]
LABELS["start-box-face-recovery"] = "Mixed v1.5.2.36 · 박스 4면 복구"
LABELS["prepare-box-face-recovery"] = LABELS["start-box-face-recovery"]
LABELS["start-small-return-filter"] = "Mixed v1.5.2.37 · 작은 점 RGB 확인"
LABELS["prepare-small-return-filter"] = LABELS["start-small-return-filter"]
LABELS["start-marker-first-ends"] = "Mixed v1.5.2.38 · 마커 우선·양끝 벽 제외"
LABELS["prepare-marker-first-ends"] = LABELS["start-marker-first-ends"]
LABELS["start-unvisited-priority"] = "Mixed v1.5.2.39 · 미탐색 우선"
LABELS["prepare-unvisited-priority"] = LABELS["start-unvisited-priority"]
LABELS["start-region-transit"] = "Mixed v1.5.2.41 · 단서 확인 후 미탐색 복귀"
LABELS["prepare-region-transit"] = LABELS["start-region-transit"]
LABELS["start-box-safe-views"] = "Mixed v1.5.2.40 · 박스 안전 관찰"
LABELS["prepare-box-safe-views"] = LABELS["start-box-safe-views"]
# Both commands use the same frozen runtime and camera profile.
for suffix, workspace, label in (
    ("safe-frontier", "v15242_safe_frontier_640_1124_ws", "Mixed v1.5.2.42 · 안전한 미탐색 복귀"),
    ("wall-jogs", "v15267_wall_jogs_lidar5_640_1127_ws", "Mixed v1.5.2.67 · 긴 벽 단차 면 + 막힘 탈출 · LiDAR 5m (실험 · 시뮬 검증 중)"),
    ("face-normals", "v15266_face_normals_lidar5_640_1127_ws", "Mixed v1.5.2.66 · 면 방향 보정 + 벽에서 떨어져 주행 · LiDAR 5m (실험 · 시뮬 검증 중)"),
    ("v155-far-wall-slot", "v155_far_wall_slot_lidar12_1127_ws", "v1.5.5 · 먼 벽 슬롯 · SLAM 12m (RSH-17 릴리스 패키지 · 실험)"),
    ("wall-time", "v15265_wall_time_lidar5_640_1127_ws", "Mixed v1.5.2.65 · 흰 다리 끔 + 벽 순회 연장 · LiDAR 5m (실험 · 시뮬 검증 중)"),
    ("lap-resume", "v15264_lap_resume_lidar5_640_1127_ws", "Mixed v1.5.2.64 · 끊긴 벽 정지점 재개 · LiDAR 5m (실험 · 시뮬 검증 중)"),
    ("unswept-sides", "v15263_unswept_sides_lidar5_640_1127_ws", "Mixed v1.5.2.63 · 안 본 상자 면 먼저·벽 표면 제외 · LiDAR 5m (실험 · 시뮬 검증 중)"),
    ("far-wall", "v152621_far_wall_lidar5_640_1127_ws", "Mixed v1.5.2.62.1 · 벽에서 떨어져 주행 · LiDAR 5m (실험)"),
    ("same-view", "v15262_same_view_lidar5_640_1127_ws", "Mixed v1.5.2.62 · 같은 자리 반복 관찰 금지 · LiDAR 5m (실험)"),
    ("swept-face", "v15261_swept_face_lidar5_640_1127_ws", "Mixed v1.5.2.61 · 훑은 면 잠시 제외 · LiDAR 5m (실험)"),
    ("box-angle", "v15260_box_angle_lidar5_640_1127_ws", "Mixed v1.5.2.60 · 비스듬한 상자 면 방향 · LiDAR 5m (실험)"),
    ("open-escape", "v15259_open_escape_lidar5_640_1127_ws", "Mixed v1.5.2.59 · 막힌 자리 열린 쪽 탈출 · LiDAR 5m (실험)"),
    ("bump-memory", "v15258_bump_memory_lidar5_640_1127_ws", "Mixed v1.5.2.58 · 범퍼 접촉 기억 · LiDAR 5m (실험)"),
    ("short-lap", "v15257_short_lap_lidar5_640_1126_ws", "Mixed v1.5.2.57 · 벽 보기 + 짧은 순회 · LiDAR 5m (실험)"),
    ("wall-departure", "v15248_wall_departure_lidar5_640_1124_ws", "Mixed v1.5.2.48 · 벽띠 연속 순회 · LiDAR 5m (실험)"),
    ("wall-entry", "v15247_wall_entry_lidar5_640_1124_ws", "Mixed v1.5.2.47 · 실측 벽 진입·순회 · LiDAR 5m (실험)"),
    ("wall-lap", "v15246_wall_lap_r2_lidar5_640_1124_ws", "Mixed v1.5.2.46 · 빠른 벽 순회 → 가운데 박스 · LiDAR 5m (실험)"),
    ("interior-first", "v15245_interior_first_lidar5_640_1124_ws", "Mixed v1.5.2.45 · 가운데 박스 우선 · LiDAR 5m (실험)"),
    ("box-continuity", "v15244_box_continuity_r1_640_1124_ws", "Mixed v1.5.2.44 · 박스 관찰 이어보기 (실험)"),
    ("sweep-mapping", "v15243_1_sweep_mapping_640_1124_ws", "Mixed v1.5.2.43.1 · 출발 회전 지도 갱신"),
    ("pending-marker", "v15243_pending_marker_640_1124_ws", "Mixed v1.5.2.43 · 7px 후보 재관찰 (실험)"),
):
    for operation in ("prepare", "start"):
        key = f"{operation}-{suffix}"
        START_WORKSPACES[key] = workspace
        LABELS[key] = label
# Default choices use 5m mapping. Previous frozen ranges remain selectable.
for suffix in ("box-safe-views", "box-continuity"):
    for operation in ("start", "prepare"):
        action = f"{operation}-{suffix}"
        previous = action + "-12m"
        START_WORKSPACES[previous] = START_WORKSPACES[action]
        LABELS[previous] = LABELS[action] + " · LiDAR 12m (이전)"
        START_WORKSPACES[action] = START_WORKSPACES[action].replace("_640_", "_lidar5_640_")
        LABELS[action] += " · LiDAR 5m"
ZONE_WORKSPACES = {START_WORKSPACES[key] for key in ("start-glass-wall", "start-white-legs", "start-chair-strip")}


FOCUS_CAMERA_ACTIONS = {"start-focus", "prepare-focus", "start-wall-first", "prepare-wall-first",
                        "start-sweep31", "prepare-sweep31", "start-chair31", "prepare-chair31",
                        "start-glass-wall", "prepare-glass-wall", "start-white-legs", "prepare-white-legs",
                        "start-chair-strip", "prepare-chair-strip",
                        "start-auto-chair-strip", "prepare-auto-chair-strip",
                        "start-box-face-recovery", "prepare-box-face-recovery",
                        "start-small-return-filter", "prepare-small-return-filter",
                        "start-marker-first-ends", "prepare-marker-first-ends",
                        "start-unvisited-priority", "prepare-unvisited-priority",
                        "start-box-safe-views", "prepare-box-safe-views",
                        "start-region-transit", "prepare-region-transit",
                        "start-safe-frontier", "prepare-safe-frontier", "start-pending-marker", "prepare-pending-marker",
                        "start-sweep-mapping", "prepare-sweep-mapping",
                        "start-box-continuity", "prepare-box-continuity",
                        "start-box-safe-views-12m", "prepare-box-safe-views-12m",
                        "start-box-continuity-12m", "prepare-box-continuity-12m",
                        "start-interior-first", "prepare-interior-first",
                        "start-wall-lap", "prepare-wall-lap",
                        "start-wall-entry", "prepare-wall-entry",
                        "start-wall-departure", "prepare-wall-departure"}
# .57 and later LiDAR 5 m Mixed menus belong to the same lineage: one RGB copy over the wired link
# (relayed to the detector on the VM), three public markers, distant hints off. Without the relay a
# 640x480 stream reached each subscriber separately and went stale (2026-11-27).
FOCUS_CAMERA_ACTIONS |= {f"{operation}-{suffix}" for operation in ("start", "prepare") for suffix in (
    "short-lap", "bump-memory", "open-escape", "box-angle", "swept-face", "same-view", "far-wall",
    "unswept-sides", "lap-resume", "wall-time", "face-normals", "wall-jogs")}
# Packages from other lineages: relay one RGB copy (640x480 needs it on the 100 Mbps link) but keep
# their own runtime settings (no three-marker or distant-hint environment).
CAMERA_RELAY_ACTIONS = {"start-v155-far-wall-slot", "prepare-v155-far-wall-slot"}


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


def drive_timing(events, phase, stopped_at, now, limit_s):
    started = next((row for row in events if row.get("event") == "mission_started"), None)
    if started is None:
        return {"elapsed_s": None, "preparation_s": None, "limit_s": limit_s, "finished": False}
    began = float(started["wall_epoch_s"])
    finished = phase not in ("queued", "preflight", "starting", "prepared", "running", "stopping")
    ending = next((row for row in reversed(events) if row.get("event") in ("stopped", "mission_finished")), None)
    end = float(ending["wall_epoch_s"]) if finished and ending else stopped_at if finished else now
    return {"elapsed_s": round(max(0., end - began), 1),
            "preparation_s": round(float(started["elapsed_s"]), 1),
            "limit_s": limit_s, "finished": finished}


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
        # Keep legacy API-only actions out of the operator menu.
        hidden = {"start", "start-r32", "start-marker-relook-045"}
        preferred = ["start-box-safe-views", "start-wall-departure", "start-wall-entry", "start-wall-lap", "start-interior-first", "start-box-continuity", "start-sweep-mapping", "start-pending-marker", "start-safe-frontier", "start-region-transit", "start-unvisited-priority", "start-marker-first-ends", "start-focus", "start-sweep31", "start-small-return-filter", "start-box-face-recovery", "start-auto-chair-strip", "start-chair-strip", "start-white-legs", "start-glass-wall", "start-chair31",
                     "start-wall-first", "start-balanced"]
        order = preferred + [key for key in START_WORKSPACES
                             if key not in hidden and key not in preferred and not key.startswith("prepare-")]
        self.policies = [{"action": key, "label": LABELS[key]} for key in order
                         if (self.workspace.parent / START_WORKSPACES[key] / "install/setup.bash").is_file()]

    def read_state(self):
        try:
            return json.loads((self.state_dir / "session.json").read_text())
        except (OSError, ValueError):
            return {}

    def glass_wall_path(self, state):
        if Path(state.get("workspace", "")).name not in ZONE_WORKSPACES:
            raise RuntimeError("유리벽 지정은 Glass Wall 정책 세션에서만 가능합니다.")
        if state.get("phase") not in ("starting", "prepared", "running") or not supervisor_alive(state):
            raise RuntimeError("실행 중인 Glass Wall 준비·주행 세션이 없습니다.")
        output = Path(state.get("output", "")).resolve()
        if not output.is_relative_to((self.repo / "run_logs/robot").resolve()):
            raise RuntimeError("유리벽 기록 경로가 주행 로그 밖입니다.")
        return output / "glass_wall.json"

    def set_glass_wall(self, payload):
        with self.lock:
            path = self.glass_wall_path(self.read_state())
            try:
                current_data = json.loads(path.read_text())
                current = current_data.get("segments", [])
                zones = current_data.get("zones", [])
            except (OSError, ValueError, KeyError):
                current = []
                zones = []
            if payload == {"clear": True}:
                segments = []
                zones = []
            elif set(payload) == {"zone"}:
                if len(zones) >= 4 or not 3 <= len(payload["zone"]) <= 12:
                    raise ValueError("구역 수 또는 꼭짓점 수가 허용 범위를 벗어났습니다.")
                vertices = tuple(tuple(float(value) for value in point) for point in payload["zone"])
                if any(len(point) != 2 or not all(map(math.isfinite, point)) for point in vertices):
                    raise ValueError("지도 좌표가 올바르지 않습니다.")
                if max(math.dist(a, b) for a in vertices for b in vertices) > 20.:
                    raise ValueError("구역 크기가 너무 큽니다.")
                twice_area = abs(sum(a[0] * b[1] - b[0] * a[1]
                                     for a, b in zip(vertices, vertices[1:] + vertices[:1])))
                if not .1 <= twice_area <= 80.:
                    raise ValueError("구역 면적이 허용 범위를 벗어났습니다.")
                def orientation(a, b, c):
                    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
                edges = list(zip(vertices, vertices[1:] + vertices[:1]))
                for first, (a, b) in enumerate(edges):
                    for second, (c, d) in enumerate(edges):
                        if second <= first + 1 or first == 0 and second == len(edges)-1:
                            continue
                        if orientation(a, b, c)*orientation(a, b, d) < 0. and orientation(c, d, a)*orientation(c, d, b) < 0.:
                            raise ValueError("구역 경계가 서로 교차합니다. 모서리를 순서대로 선택하세요.")
                segments = current
                zones = [*zones, vertices]
            else:
                if set(payload) != {"points"} or len(payload["points"]) != 2:
                    raise ValueError("두 끝점이 필요합니다.")
                points = tuple(tuple(float(value) for value in point) for point in payload["points"])
                if any(len(point) != 2 or not all(map(math.isfinite, point)) for point in points):
                    raise ValueError("지도 좌표가 올바르지 않습니다.")
                if not .2 <= math.dist(*points) <= 20. or len(current) >= 8:
                    raise ValueError("유리벽 길이 또는 구간 수가 허용 범위를 벗어났습니다.")
                segments = [*current, points]
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_name(path.name + "." + secrets.token_hex(4) + ".tmp")
            temporary.write_text(json.dumps({"frame": "map", "segments": segments, "zones": zones}) + "\n")
            temporary.replace(path)
            return self.snapshot()

    def command(self, action):
        workspace = self.workspace
        preparing = action.startswith("prepare-")
        share_camera = action in FOCUS_CAMERA_ACTIONS or action in CAMERA_RELAY_ACTIONS
        forward_045 = action == "start-marker-relook-045"
        recover_stall = action in ("start-stall-recovery", "start-crowded")
        native = action in ("start-upstream-r32", "prepare-upstream-r32")
        side_inspection = action in ("start-side", "start-general")
        if START_WORKSPACES.get(action):
            workspace = self.workspace.parent / START_WORKSPACES[action]
            action = "prepare" if preparing else "start"
        command = [sys.executable, "-u", str(self.repo / "scripts/robot_session.py"), action,
                   "--repo", str(self.repo), "--workspace", str(workspace),
                   "--domain", str(self.domain), "--ntp-server", self.ntp_server,
                   "--duration", str(self.duration)]
        if share_camera:
            command.append("--share-camera")
        if forward_045:
            command.append("--forward-045")
        if recover_stall:
            command.append("--recover-stall")
        if side_inspection:
            command.append("--side-inspection")
        if native:
            return ["bash", "-c", 'source /opt/ros/jazzy/setup.bash && source "$1/install/setup.bash" && shift && exec "$@"',
                    "r32-board", str(workspace), *command, "--runtime", "upstream-r32"]
        if action in ("start", "prepare"):
            command = ["bash", str(self.repo / "run_policy.sh"), "--mode", "robot",
                       "--workspace", str(workspace), "--policy",
                       str(self.repo / "policy_releases/2026-11-13_face_intent_v2/policy.json"),
                       "--", *command]
        return command

    def spawn(self, action):
        path = self.state_dir / ("board-" + action + "-" + secrets.token_hex(6) + ".log")
        profile = self.repo / "robot_profiles/wired_flow_v3/fastdds_vm.xml"
        env = dict(os.environ, ROS_DOMAIN_ID=str(self.domain),
                   RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
                   FASTRTPS_DEFAULT_PROFILES_FILE=str(profile),
                   OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
        if action in FOCUS_CAMERA_ACTIONS:
            env.update(OSP_PUBLIC_MARKER_COUNT="3", OSP_SURVEY_DISTANT_HINTS="false")
        if (action in FOCUS_CAMERA_ACTIONS or action in CAMERA_RELAY_ACTIONS
                or action in ("start-balanced", "prepare-balanced")):
            env["OSP_SHARE_CAMERA"] = "1"
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
                          "prepared": "준비 완료 · 출발 버튼을 기다리는 중",
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
                translations = {"create3_data_missing": "Create3 바퀴·안전 데이터가 없습니다. 본체와 연결을 확인하세요. 주행은 시작하지 않았습니다.",
                                "create3_clock_stale": "Create3 바퀴 시각이 오래됐습니다. 본체 시각 동기화 후 다시 준비하세요. 주행은 시작하지 않았습니다.",
                                "robot_wired_link_unreachable": "로봇 유선 연결에 응답이 없습니다. 전원·랜선·라우터를 확인하세요. 주행은 시작하지 않았습니다.",
                                "robot_wired_interface_reconnect_failed": "VM 유선 연결 복구에 실패했습니다. 주행은 시작하지 않았습니다.",
                                "robot_wired_interface_restore_failed": "VM 유선 포트를 다시 켜지 못했습니다. 주행은 시작하지 않았습니다.",
                                "waiting_for_scan": "LiDAR 데이터를 받지 못했습니다. 로봇 센서 연결을 확인하세요.",
                                "waiting_for_camera": "카메라 데이터를 받지 못했습니다.",
                                "robot_docked": "로봇이 도크에 있습니다. 도크 밖으로 이동한 뒤 구동하세요.",
                                "active_hazard_types": "범퍼·낭떠러지·바퀴 상태를 확인하세요.",
                                "robot_time_server_unreachable": "로봇 유선 연결이 끊겨 시간 서버에 닿지 못했습니다. VM 유선 브리지와 로봇 연결을 확인하세요.",
                                "clock": "시간 동기화를 확인하지 못해 출발하지 않았습니다."}
                detail = next((text for key, text in translations.items() if key in error), error)
            busy = phase in ("queued", "preflight", "starting", "prepared", "running", "stopping")
            events = []
            if state.get("output") and (alive or belongs):
                try:
                    lines = (Path(state["output"]) / "startup.jsonl").read_text().splitlines()
                except OSError:
                    lines = []
                for line in lines:
                    try:
                        events.append(json.loads(line))
                    except ValueError:
                        continue
            try:
                stopped_at = (self.state_dir / "session.json").stat().st_mtime
            except OSError:
                stopped_at = time.time()
            timing = drive_timing(events, phase, stopped_at, time.time(), self.duration)
            selected = self.selected
            if belongs and state.get("workspace"):
                name = Path(state["workspace"]).name
                selected = next((key for key, value in START_WORKSPACES.items() if value == name), "start")
                if selected == "start-marker-relook" and state.get("forward_045"):
                    selected = "start-marker-relook-045"
            glass_segments = []
            glass_zones = []
            if Path(state.get("workspace", "")).name in ZONE_WORKSPACES:
                try:
                    annotations = json.loads((Path(state["output"]) / "glass_wall.json").read_text())
                    glass_segments = annotations.get("segments", [])
                    glass_zones = annotations.get("zones", [])
                except (OSError, ValueError, KeyError):
                    pass
            return dict(policy=LABELS.get(selected), enabled=True, token=self.token, phase=phase, detail=detail,
                        can_start=not busy, can_go=phase == "prepared" and alive,
                        can_stop=True, policies=self.policies, events=events[-8:], timing=timing,
                        glass_wall_segments=glass_segments, glass_wall_zones=glass_zones)

    def request(self, action):
        with self.lock:
            if action == "go":
                state = self.read_state()
                if self.snapshot()["phase"] != "prepared" or not state.get("output"):
                    raise RuntimeError("준비가 완료된 실물 세션이 없습니다.")
                (Path(state["output"]) / "go.request").touch(exist_ok=False)
                return self.snapshot()
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
