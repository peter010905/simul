"""Live, human-readable timeline of what the policy did and why (read-only).

The board runs on the same VM as the robot session. Every Mixed policy already writes its
decisions to the run's artifacts directory; this module tails those JSONL files incrementally
and turns each record into one Korean line. It never writes to the run directory.
"""
import json
from pathlib import Path

DECISIONS = {
    'quick_perimeter_pass': ('벽 순회', '다음 벽 정지점으로 이동'),
    'waiting_for_safe_perimeter': ('벽 순회 대기', '안전한 벽 정지점을 찾는 중'),
    'resume_interrupted_lap_stop': ('벽 순회', '끊겼던 벽 정지점으로 다시 이동'),
    'regional_camera_gain_per_route': ('탐색', '이동 거리 대비 카메라로 새로 볼 영역이 큰 곳으로 이동'),
    'interior_first_exploration': ('탐색', '아직 가 보지 않은 내부 공간으로 이동'),
    'finish_current_box': ('상자', '같은 상자의 남은 면을 보러 이동'),
    'resume_interrupted_face': ('상자', '끊겼던 상자 면을 다시 보러 이동'),
    'box_safe_retry': ('상자', '막혔던 상자 면을 다른 자리에서 재시도'),
    'accepted_marker_followup': ('마커', '카메라에 보인 마커를 더 가까이서 확인'),
    'verified_marker_approach': ('마커', '확인된 마커에 1.5 m 안으로 접근'),
    'route_to_confirmed_print_side': ('마커', '확인된 마커 앞쪽으로 경로 이동'),
    'wall_marker_evidence': ('마커', '벽에서 본 마커 단서를 확인하러 이동'),
    'local_print_candidate': ('마커', '근처 인쇄물 후보를 확인'),
    'white_leg_candidate': ('흰 다리 후보', '카메라 흰 점 + 가는 장애물을 확인하러 이동'),
    'region_leave': ('구역', '오래 머문 구역을 떠나 다른 구역으로 이동'),
    'region_resume': ('구역', '덜 본 구역으로 돌아가 탐색 재개'),
    'region_escape': ('구역', '막힌 구역에서 빠져나감'),
    'region_transit': ('구역', '다른 구역으로 이동'),
    'safe_frontier_fallback': ('탐색', '갈 수 있는 미탐색 경계로 이동'),
    'interior_tasks_temporarily_unavailable': ('대기', '지금 갈 수 있는 목표가 없음'),
    'mission_start': ('시작', '미션 시작'),
}
LAP_REASONS = {
    'perimeter_time_budget': '벽 순회 시간이 끝나 내부 탐색으로 전환',
    'closed_perimeter_pass': '벽 한 바퀴 완료, 내부 탐색으로 전환',
    'no_safe_perimeter_goal': '안전한 벽 진입점이 없어 내부 탐색으로 전환',
    'perimeter_continuation_unavailable': '벽을 따라 더 갈 곳이 없어 내부 탐색으로 전환',
}
FACE_EVENTS = {
    'face_dispatched': '상자 면 관찰 시작',
    'sweep_finished': '상자 면 훑기 완료',
    'face_failed': '상자 면 관찰 실패',
    'face_suspended': '상자 면 관찰을 잠시 멈춤',
    'resume_limit': '상자 면 재개 한도 도달',
}
FAIL_REASONS = {
    'no_progress_10s': '10초 동안 진전 없음', 'navigation_stalled': '이동이 막힘',
    'near_pass_inspection': '지나가며 본 단서 확인으로 전환', 'marker_hint': '마커 단서가 보임',
    'readable_marker': '읽을 수 있는 마커가 보임', 'registered_marker': '마커 등록됨',
    'navigation_failed': '이동 실패', 'new_marker_hint': '새 마커 단서', 'replan': '경로 재계획',
    'mission_finished': '미션 종료',
}
SESSION_EVENTS = {
    'mission_started': '미션 시작', 'first_motion': '첫 움직임', 'mission_finished': '미션 종료',
    'stopped': '정지 확인', 'go_requested': '출발 요청', 'prepared': '주행 준비 완료',
}
FILES = ('center_wall.jsonl', 'box_continuity.jsonl', 'marker_directed_looks.jsonl', 'startup.jsonl')


def _seconds(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _entry(t, kind, title, detail=''):
    return dict(t=None if t is None else round(t, 1), kind=kind, title=title, detail=detail)


class DecisionLog:
    def __init__(self, limit=400):
        self.limit = limit
        self.root = None
        self.offsets = {}
        self.entries = []
        self.mission_start = None
        self.lap_phase = None
        self.last_key = None
        self.last_decision = None
        self.noted_ids = set()

    def follow(self, artifacts):
        root = Path(artifacts) if artifacts else None
        if root != self.root:
            self.root, self.offsets, self.entries = root, {}, []
            self.mission_start = self.lap_phase = self.last_key = self.last_decision = None
            self.noted_ids = set()

    def _lines(self, name):
        path = self.root / name
        try:
            with path.open('rb') as stream:
                stream.seek(self.offsets.get(name, 0))
                data = stream.read()
        except OSError:
            return []
        end = data.rfind(b'\n') + 1      # keep a half-written last line for the next poll
        self.offsets[name] = self.offsets.get(name, 0) + end
        rows = []
        for line in data[:end].splitlines():
            try:
                row = json.loads(line)
            except ValueError:
                continue
            if isinstance(row, dict):
                rows.append(row)
        return rows

    def _decision(self, row):
        t = _seconds(row.get('elapsed'))
        out = []
        lap = row.get('wall_lap') or {}
        if lap.get('phase') != self.lap_phase:
            if self.lap_phase == 'perimeter' and lap.get('phase') == 'interior':
                detail = f"{lap.get('path_m', 0)} m · 정지점 {lap.get('confirmed_stops', 0)}곳"
                out.append(_entry(t, 'phase', LAP_REASONS.get(lap.get('reason'), '벽 순회 종료: ' + str(lap.get('reason'))), detail))
            self.lap_phase = lap.get('phase')
        reason = row.get('reason') or ''
        title, detail = DECISIONS.get(reason, ('결정', reason))
        extra = []
        if row.get('marker_id') is not None:
            extra.append(f"마커 #{row['marker_id']}")
        if reason == 'quick_perimeter_pass' and lap:
            extra.append(f"순회 {lap.get('path_m', 0)} m · 정지점 {lap.get('confirmed_stops', 0)}곳")
        text = ' · '.join([detail] + extra)
        key = (reason, row.get('marker_id'))
        if key == self.last_key and self.last_decision is not None and not out:
            # The same decision again: one line with a count; its detail shows the latest values.
            self.last_decision['repeat'] = self.last_decision.get('repeat', 1) + 1
            self.last_decision['t_last'] = None if t is None else round(t, 1)
            self.last_decision['detail'] = text
            return out
        self.last_key = key
        self.last_decision = _entry(t, 'decision', title, text)
        out.append(self.last_decision)
        return out

    def _face(self, row):
        event = row.get('event')
        if event not in FACE_EVENTS:
            return []
        detail = FAIL_REASONS.get(row.get('reason'), row.get('reason') or '')
        if row.get('resumed'):
            detail = (detail + ' · ' if detail else '') + '끊긴 면 재개'
        return [_entry(_seconds(row.get('elapsed')), 'face', FACE_EVENTS[event], detail)]

    def _look(self, row):
        event, kind = row.get('event'), row.get('kind')
        t = _seconds(row.get('stamp'))
        marker = f"마커 #{row['id']}" if row.get('id') is not None else ''
        if event == 'selected' and kind in ('focus', 'approach'):
            return [_entry(t, 'marker', '마커 쪽으로 카메라 맞춤' if kind == 'focus' else '마커 접근 시작', marker)]
        if event == 'cleared' and row.get('reason') == 'registered_marker':
            return [_entry(t, 'found', '마커 등록', marker)]
        return []

    def _session(self, row):
        event = row.get('event')
        raw = _seconds(row.get('elapsed_s'))
        if event == 'mission_started' and raw is not None:
            self.mission_start = raw
        t = None if raw is None or self.mission_start is None else raw - self.mission_start
        if event == 'bumper_recovery':
            return [_entry(t, 'warn', f"범퍼 접촉 {row.get('contacts', '?')}회",
                           '미션을 멈추고 뒤로 물러난 뒤 다시 계획')]
        if event == 'safety_stop':
            return [_entry(t, 'warn', '안전 정지', str(row.get('reason', '')))]
        if event in SESSION_EVENTS:
            return [_entry(t, 'session', SESSION_EVENTS[event], str(row.get('reason', '') or ''))]
        return []

    def poll(self):
        if self.root is None:
            return
        for name, handler in zip(FILES, (self._decision, self._face, self._look, self._session)):
            for row in self._lines(name):
                self.entries.extend(handler(row))
        self.entries.sort(key=lambda e: (e['t'] is None, e['t'] if e['t'] is not None else 0.))
        del self.entries[:-self.limit]

    def note_markers(self, ids, elapsed):
        """Marker IDs the board saw confirmed on /targets, at the mission's elapsed time."""
        new = sorted(set(ids) - self.noted_ids)
        self.noted_ids.update(new)
        if new and self.root is not None:
            self.entries.extend(_entry(_seconds(elapsed), 'found', '마커 발견', f'마커 #{i}') for i in new)
            self.entries.sort(key=lambda e: (e['t'] is None, e['t'] if e['t'] is not None else 0.))

    def snapshot(self, newest=150):
        return dict(run=self.root.parent.name if self.root else None,
                    entries=list(reversed(self.entries[-newest:])))


def artifacts_dir(state, run_logs):
    """The running or last session's artifacts directory, kept inside run_logs."""
    run_logs = Path(run_logs).resolve()
    output = state.get('output') if isinstance(state, dict) else None
    if output:
        path = Path(output).resolve()
        if path.is_relative_to(run_logs) and path.is_dir():
            return path
    runs = [p for p in run_logs.glob('*/artifacts') if p.is_dir()] if run_logs.is_dir() else []
    return max(runs, key=lambda p: p.stat().st_mtime).resolve() if runs else None
