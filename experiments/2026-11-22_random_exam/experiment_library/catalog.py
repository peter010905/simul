"""Index the shared simulation board by the controller actually used on each drive."""
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile
import threading
import time


FOCUS = 'Mixed v1.5.2.1 · 마커 재관찰 유지'
SURVEY = 'Mixed v1.5.2 · Visual Survey'
BALANCED = 'Mixed v1.5.1-balanced'
PRINT = 'Mixed · Balanced Print'
SOURCES = {
    'dfd33c79d93498337a7f4257900f724894e7b5048f2dfee67a39ab395b3691bd': FOCUS,
    '46a6dd68c5070172a22b6680f2945cf22aa8b4f3866797316c820e9fb82e135a': SURVEY,
    'afbaaaaf882c32b41579e8614997c70d0fe6721626df9518370ffe156ad85595': BALANCED,
    '5152baac3122da4e09fd15e982509df78bfee74891827d8374a11d201d12a0bb': PRINT,
    'c8366100c1a3dd3955ccbea4b17f7d010fd9c915e0d7c0083c2d2acb84922444': 'Mixed · Adaptive Survey r2',
    'eb144ce44ef1d2fb5ffeb158d80e012f97eb9fbce273ebfcbf9505932073fee2': 'Mixed · Surface Followthrough r5',
}
STAGES = {
    'v152_weak_wall_priority_1123_ws': 'Mixed v1.5.2.11 · 먼 벽 후보 우선',
    'flow_v151_exposure_sync_r2_ws': BALANCED, 'v151_balanced_print_r1_1122_ws': PRINT,
    'v152_visual_survey_r3_1122_ws': SURVEY, 'v152_focus_continuity_box1122_ws': FOCUS,
    'v151_surface_followthrough1122_r5_clean_ws':'Mixed · Surface Followthrough r5',
    'v151_adaptive_survey1122_r2_ws':'Mixed · Adaptive Survey r2',
    'center_first_3b3dbbc':'Mixed · 중앙 장애물 우선',
    'flow_v3_r32_adaptive_ws':'Mixed · 공간 맞춤 면 관찰',
    'flow_v3_r32_shared_face_final_ws':'Mixed · 상자 면 관찰 이력 공유',
    'flow_v3_r32_box_batch_ws':'Mixed · 상자 묶음 관찰',
    'flow_v3_r32_cluster_exit_ws':'Mixed · 상자 군집 이탈',
    'flow_v3_r32_wall_cues_final_ws':'Mixed · 벽 후보 기억',
}
LEGACY = {
    'v151-boxes5':BALANCED, 'v151':BALANCED,
    'v151-rescue360':'Mixed v1.5.1 · Rescue360', 'v151-balance':'Mixed v1.5.1 · 방위별 벽 관찰',
    'v151-survey':'Mixed v1.5.1 · 미방문 구역 이동', 'v151-perimeter':'Mixed v1.5.1 · 외곽 순회',
    'v151-perimeter-r3':'Mixed v1.5.1 · 외곽 순회 r3', 'v152-wall':'Mixed v1.5.2 · 이동 중 벽 관찰',
    'v152-wall-r2':'Mixed v1.5.2 · 이동 중 벽 관찰 r2', 'v153-region':'Mixed v1.5.3 · 구역 전환',
    'r32-latest-d8704f6':'원본 R32 · d8704f6', 'r32':'원본 R32 · 리비전 미기록',
    'flow':'Flow v3', 'mixed':'Mixed · 세부 버전 미기록', 'mixed-baseline':'Mixed · 세부 버전 미기록',
}


def version(label, revision, signature):
    if signature in SOURCES:
        return SOURCES[signature]
    # A new source in an existing workspace is an independent revision.
    if signature:
        return (revision.split(' · ')[-1] if revision else '실험 정책') + ' · '+signature[:8]
    if revision in STAGES:
        return STAGES[revision]
    for word in revision.replace(';',' ').split():
        if word in STAGES:
            return STAGES[word]
    if revision.startswith('mixed-box-views-'):
        return 'Mixed · 상자 면 관찰 '+revision.rsplit('-',1)[-1]
    if revision.startswith('5e95333'):
        return '원본 R32 Resume · 5e95333'
    if label.startswith('v12-'):
        return 'Mixed v1.2 · 카메라 실험'
    if label.startswith('v15-'):
        stage = label.split('-')[1]
        return 'Mixed v1.5 · '+{'baseline':'Baseline','camera':'Camera','candidate':'Candidate',
                               'committed':'Committed','rescue360':'Rescue360','sync':'Sync'}.get(stage,stage)
    family = label.split('-track-')[0]
    if family in LEGACY:
        return LEGACY[family]
    if re.fullmatch(r'(flow|mixed)-\d+',label):
        return LEGACY[label.split('-')[0]]
    return revision or '버전 미확인'


def timestamp(path):
    try:
        return path.stat().st_mtime_ns
    except OSError:
        return 0


def recording_folder(roots,label,fallback):
    return next((root/label for root in roots if root and (root/label).is_dir()),fallback)


def read(path):
    try:
        return json.loads(path.read_text())
    except (OSError,ValueError):
        return {}


class Library:
    def __init__(self, board, args):
        self.board = board
        self.roots = [p for p in (args.track_source,args.cards.parent/'recordings',args.source) if p]
        self.state = args.cards.parent/'experiment-library-visibility.json'
        self.overrides = json.loads(self.state.read_text()) if self.state.is_file() else {}
        self.lock = threading.RLock()
        self.meta = {}
        self.finished = {}
        self.rows = {}
        self.live = {}
        self.refresh_at = 0.

    def metadata(self, run):
        source = recording_folder(self.roots,run.label,run.path)
        rev = source/'policy_revision.txt'
        manifest = source/'policy_manifest.json'
        scene = source/'scene.json'
        if not scene.is_file():
            scene = run.path/'scene.json'
        key = (source,timestamp(rev),timestamp(manifest),timestamp(scene))
        cached = self.meta.get(run.id)
        if cached and cached[0] == key:
            return cached[1]
        geometry = read(scene)
        revision = rev.read_text(errors='replace').strip() if key[1] else ''
        files = read(manifest)
        signature = hashlib.sha256(json.dumps(files,sort_keys=True,separators=(',',':')).encode()).hexdigest() if files else ''
        cfg = geometry.get('randomization',{})
        bodies = geometry.get('obstacles',[])
        # The public exam fixes the desks, while the number and placement of
        # extra boxes and marker hosts vary. Keep these valid random draws in
        # the visible experiment list even when all three markers land on walls.
        legacy_layout = ((cfg.get('obstacles'),len(bodies)) in
                         (('spread_exam_boxes',3),('exam_boxes',5)))
        random_exam = cfg.get('generator')=='exam-random-layout' and len(bodies)<=5
        official = (cfg.get('furniture')=='fixed' and (legacy_layout or random_exam)
                    and all(b.get('shape')=='box' for b in bodies)
                    and len(geometry.get('targets',[]))==3)
        started = next((t for t in (timestamp(source/'camera/00001.jpg'),timestamp(rev),
                                    timestamp(run.path/'manifest.json')) if t),0)
        value = dict(version=version(run.label,revision,signature),official=official,seed=geometry.get('seed'),
                     total=len(geometry['targets']) if isinstance(geometry.get('targets'),list) else None,
                     boxes=len(bodies) if official else None,started=started/1e9,source=source)
        self.meta[run.id] = (key,value)
        return value

    def row(self, run, now):
        meta = self.metadata(run)
        evaluation = run.path/'simulation_evaluation.json'
        stamp = timestamp(evaluation)
        sealed = (stamp,timestamp(run.path/'mission_results/flow/latest.json'),self.meta[run.id][0])
        previous = self.finished.get(run.id)
        if stamp and previous and previous[0]==sealed:
            return previous[1],None
        card = run.summary(now)
        source = meta['source']
        phase = card['phase']
        last = timestamp(source/'trajectory.csv')/1e9 or timestamp(run.path/'simulation_trace.jsonl')/1e9
        if phase in ('running','stalled') and last and now-last>120:
            phase='stalled'
        if phase=='waiting' and (card.get('start_abandoned') or now-meta['started']>180):
            phase='unstarted'
        live = phase in ('running','waiting') or (phase=='stalled' and last>0 and now-last<=120)
        score = read(evaluation).get('exam_score',{}) if stamp else {}
        confirmed = score.get('registered',len(card['confirmed']))
        approached = score.get('verified',len(card['approached']))
        item = {k:v for k,v in meta.items() if k!='source'}
        item.update(id=run.id,label=run.label,phase=phase,live=live,confirmed=confirmed,approached=approached,
                    sim_time=score.get('elapsed_s',card['sim_time']),bumps=card['bumper_events'],strict=score.get('strict'))
        if stamp and phase=='finished':
            self.finished[run.id]=(sealed,item)
        if live:
            card=dict(card,phase=phase)
        return item,card if live else None

    def refresh(self):
        now = time.time()
        if time.monotonic()<self.refresh_at:
            return
        if now-self.board.scanned>5:
            self.board.discover()
        with self.board.lock:
            runs=list(self.board.runs.values())
        rows,live={},{}
        for run in runs:
            item,card=self.row(run,now)
            rows[item['id']]=item
            if card:
                live[item['id']]=card
        self.rows,self.live=rows,live
        self.refresh_at=time.monotonic()+1.5

    def archived(self,row):
        return not row['live'] and self.overrides.get(row['id'],self.default_archive(row))

    @staticmethod
    def default_archive(row):
        return not row['official'] or row['phase'] in ('stalled','unstarted')

    def list(self, archive=False):
        with self.lock:
            self.refresh()
            rows=list(self.rows.values())
            current=sum(not self.archived(r) for r in rows)
            selected=[r for r in rows if self.archived(r)==archive]
            return dict(current=current,archived=len(rows)-current,runs=selected,live={} if archive else self.live)

    def set_archived(self, ident, archived):
        with self.lock:
            self.refresh_at=0.
            self.refresh()
            row=self.rows.get(ident)
            if row is None:
                raise KeyError(ident)
            if row['live']:
                raise ValueError('진행 중인 주행은 보관할 수 없습니다.')
            updated=dict(self.overrides)
            if archived==self.default_archive(row):
                updated.pop(ident,None)
            else:
                updated[ident]=archived
            if updated!=self.overrides:
                with tempfile.NamedTemporaryFile(mode='w',dir=self.state.parent,delete=False) as out:
                    json.dump(updated,out,sort_keys=True)
                    temp=Path(out.name)
                try:
                    os.replace(temp,self.state)
                finally:
                    temp.unlink(missing_ok=True)
                self.overrides=updated
