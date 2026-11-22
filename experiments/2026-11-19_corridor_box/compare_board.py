"""Expose these Flow Gazebo trials in the upstream sim_board read-only dashboard."""
import argparse
from bisect import bisect_right
import csv
import json
import math
import re
import os
import subprocess
from pathlib import Path
import sys
import threading
import time
from http.server import ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

import yaml
import cv2
import numpy as np


def put(path, data):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data))
    tmp.replace(path)


def numbered(root):
    return sorted(d for d in root.iterdir() if d.is_dir() and d.name.split('-')[0] in ('flow','mixed') and d.name.split('-')[-1].isdigit())


def rows(path):
    with path.open() as f:
        return list(csv.DictReader(f.read().rsplit('\n',1)[0].splitlines()))


def decision_maps(src, snapshots):
    decisions=[]
    paths=sorted((src/'native/policy_decisions').glob('*.npz'))
    for p in paths:
        with np.load(p, allow_pickle=False) as z:
            i=int(z['selected_index'])
            if i>=0:
                x,y,yaw=map(float,z['candidates'][i,:3])
                decisions.append((float(z['elapsed']),dict(x=x,y=y,yaw=yaw,kind='탐색 후보 선택')))
            if p==paths[-1]:
                dst=snapshots/('snapshot_'+p.stem)
                if not (dst/'map.pgm').exists():
                    dst.mkdir(parents=True,exist_ok=True)
                    grid=z['data']
                    pixels=np.where(grid<0,205,np.rint(254*(1-np.clip(grid,0,100)/100.))).astype('uint8')
                    meta=dict(resolution=float(z['resolution']),origin=[*map(float,z['origin'][:2]),0.0])
                    (dst/'map.yaml').write_text(yaml.safe_dump(meta))
                    cv2.imwrite(str(dst/'map.pgm'),np.flipud(pixels))
    return decisions, ('snapshot_'+paths[-1].stem if paths else '.')


def refresh(src, dst, scene, cache, origin=(6, 2.4), duration=120):
    dst.mkdir(parents=True,exist_ok=True)
    if not (dst/'scene.json').exists():
        put(dst/'scene.json',scene)
        (dst/'mission.yaml').write_text(f'duration: {duration}\n')
        put(dst/'manifest.json',{})
    exam_score = json.loads((src/'exam-score.json').read_text()) if (src/'exam-score.json').is_file() else None
    if (dst/'simulation_evaluation.json').exists():
        if exam_score is not None:
            previous=json.loads((dst/'simulation_evaluation.json').read_text())
            if previous.get('exam_score') != exam_score:
                put(dst/'simulation_evaluation.json',dict(
                    evaluation=(f"{exam_score['expected']}개 마커 등록·접근 완료"
                                if exam_score.get('strict') else '일부 미달'),
                    marker_evaluation=', '.join(str(i) for i,r in exam_score['records'].items()
                                                if r.get('registered_s') is not None) or '없음',
                    exam_score=exam_score))
        return
    if not (src/'headings.csv').is_file() or not (src/'trajectory.csv').is_file():
        return
    ox, oy = origin
    headings=rows(src/'headings.csv')
    path=rows(src/'trajectory.csv')
    if not headings or not path:
        return
    measured=json.loads((src/'metrics.json').read_text()) if (src/'metrics.json').exists() else None
    snapshots=dst/'mission_results/flow'
    snapshots.mkdir(parents=True,exist_ok=True)
    decisions,snapshot=decision_maps(src,snapshots)
    marker_files=list((src/'native').glob('*/markers.csv'))
    if src.name.startswith('r32-track-'):
        for latest in (src/'native').glob('*/latest.json'):
            manifest=json.loads(latest.read_text())
            native=latest.parent/manifest['snapshot']
            marker_files=[native/'markers.csv']
            if (native/'map.pgm').is_file() and (native/'map.yaml').is_file():
                snapshot=native.name
                reference=snapshots/snapshot
                if not reference.exists():
                    reference.symlink_to(native, target_is_directory=True)
    goal_times=[item[0] for item in decisions]
    htimes=[float(row['elapsed_sim_s']) for row in headings if row.get('elapsed_sim_s')]
    count,approached=cache.get(src.name,(0,set()))
    if src.name not in cache:
        (dst/'simulation_trace.jsonl').write_text('')
    n=min(len(headings),len(path),measured['truth_samples'] if measured else len(path))
    wall0=(src/'camera/00001.jpg').stat().st_mtime if (src/'camera/00001.jpg').exists() else time.time()
    actual_targets={t['id']:(t['x']+ox,t['y']+oy) for t in scene['targets']}
    with (dst/'simulation_trace.jsonl').open('a') as f:
        for i in range(count,n):
            row=path[i]
            t,x,y=map(float,(row['elapsed_sim_s'],row['world_x'],row['world_y']))
            ids=list(map(int,row['registered_ids'].split()))
            for ident in ids:
                if ident in actual_targets and math.dist((x,y),actual_targets[ident])<=1.5:
                    approached.add(ident)
            if i%8 and i!=n-1 and row['bumper']!='True':
                continue
            heading=headings[max(0,bisect_right(htimes,t)-1)]
            g=bisect_right(goal_times,t)-1
            frame=dict(goal=decisions[g][1] if g>=0 else None,elapsed=t,x=x-ox,y=y-oy,yaw=float(heading['yaw_rad']),confirmed_ids=ids,
                       bumper_active=row['bumper']=='True',state='주행 중')
            if i==n-1:
                frame['elapsed_wall']=measured['elapsed_wall_s'] if measured else max(0,time.time()-wall0)
                if measured: frame['state']='FINISHED'
            f.write(json.dumps(frame)+'\n')
    cache[src.name]=(n,approached)
    ids=measured['final_ids'] if measured else list(map(int,path[n-1]['registered_ids'].split()))
    if measured: approached=set(map(int,measured['registered_and_within_s']))
    found={}
    for markers in marker_files:
        found={int(r['id']):(float(r['x']),float(r['y'])) for r in rows(markers)}
    put(snapshots/'latest.json',dict(snapshot=snapshot,status=dict(reason=measured['reason'] if measured else '',
        targets=[dict(id=i,x=found.get(i,(None,None))[0],y=found.get(i,(None,None))[1],confirmed=True,approached=i in approached) for i in ids])))
    if measured:
        if n<measured['truth_samples']:
            return
        put(dst/'simulation_evaluation.json',dict(
            evaluation=(f"{exam_score['expected']}개 마커 등록·접근 완료" if exam_score.get('strict')
                        else '일부 미달') if exam_score is not None else
                       (f'{len(actual_targets)}개 마커 등록·접근 완료' if measured['pair_time_s'] is not None else '일부 미달'),
            marker_evaluation=', '.join(map(str,ids)) or '없음',
            **(dict(exam_score=exam_score) if exam_score is not None else {})))


def main():
    parser=argparse.ArgumentParser()
    for key in ('source','dashboard','cards'):
        parser.add_argument('--'+key,type=Path,required=True)
    parser.add_argument('--track-source', type=Path)
    parser.add_argument('--track-runner', type=Path)
    parser.add_argument('--host',default='127.0.0.1')
    args=parser.parse_args()
    sys.path.insert(0,str(args.dashboard))
    from sim_board import Board, make_handler
    cfg=yaml.safe_load((args.source/'2026-11-19_corridor_box/scene.yaml').read_text())
    scene=dict(room_size=[12,3],seed=cfg['seed'],obstacles=[dict(o,x=o['x']-6,y=o['y']-2.4) for o in cfg['obstacles']],
               targets=[dict(t,x=t['x']-6,y=t['y']-2.4,attached_to='복도 옆 벽' if t['id']==7 else '상자 뒷면') for t in cfg['targets']])
    args.cards.mkdir(parents=True,exist_ok=True)
    board=Board([args.cards],24*365)
    def recordings():
        for src in numbered(args.source):
            yield src, scene, (6, 2.4), 120
        if args.track_source and args.track_source.is_dir():
            for src in sorted(args.track_source.glob('*-track-*')):
                if not (src/'scene.yaml').is_file():
                    continue
                track = json.loads((src/'scene.json').read_text())
                layout = json.loads((src/'run/simulation/demo_layout.json').read_text())
                track['partitions'] = track['partitions'] + layout['tables']
                yield src, track, (0, 0), 600

    def update():
        cache={}
        while True:
            for src, geometry, origin, duration in recordings():
                try: refresh(src,args.cards/src.name,geometry,cache,origin,duration)
                except (OSError,ValueError,IndexError) as exc: print(src.name,exc,flush=True)
            time.sleep(2)
    threading.Thread(target=update,daemon=True).start()
    class Handler(make_handler(board)):
        def do_POST(self):
            if self.path != '/api/start-track' or not args.track_runner or not args.track_source:
                return self.send(404, {'error': 'unknown action'})
            expected = f'{args.host}:8780'
            if self.headers.get('Host') != expected or self.headers.get('Origin') != 'http://'+expected:
                return self.send(403, {'error': '대시보드에서 시작해 주세요.'})
            length = self.headers.get('Content-Length', '')
            if self.headers.get_content_type() != 'application/json' or not length.isdecimal() or int(length)>128:
                return self.send(400, {'error': '잘못된 주행 요청'})
            try:
                request = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                policy = request['policy']
            except (ValueError, KeyError, TypeError):
                return self.send(400, {'error': '잘못된 정책'})
            if policy not in ('flow','mixed','center','adaptive','shared','r32','pair'):
                return self.send(400, {'error': '알 수 없는 정책'})
            reference_dir = ''
            if 'layout_run_id' in request:
                run_id = request['layout_run_id']
                reference = board.get(run_id) if isinstance(run_id, str) else None
                if reference is None and isinstance(run_id, str):
                    board.discover()
                    reference = board.get(run_id)
                if not reference or '-track-' not in reference.label:
                    return self.send(400, {'error': '기준 트랙을 찾을 수 없습니다.'})
                reference_dir = str(args.track_source/reference.label)
            args.track_source.mkdir(parents=True, exist_ok=True)
            with (args.track_source/'launch-console.log').open('a') as log:
                process = subprocess.Popen([str(args.track_runner),policy], stdin=subprocess.DEVNULL,
                    stdout=log, stderr=log, start_new_session=True,
                    env=dict(os.environ, TRACK_RUNS=str(args.track_source), TRACK_REPLAY_DIR=reference_dir))
            try:
                code = process.wait(timeout=.4)
            except subprocess.TimeoutExpired:
                threading.Thread(target=process.wait, daemon=True).start()
                message = '같은 맵에서 주행을 준비하고 있습니다.' if reference_dir else '새 무작위 트랙을 준비하고 있습니다.'
                return self.send(202, {'message': message})
            return self.send(409, {'error': '이미 주행 중입니다.'} if code==75 else {'error': '실행 기록을 확인해 주세요.'})

        def do_GET(self):
            url=urlsplit(self.path)
            if url.path == '/api/run' and args.track_source:
                run = board.get(parse_qs(url.query).get('id',[''])[0])
                if run and run.label.startswith('r32-track-'):
                    folder = args.track_source/run.label
                    revision = folder/'policy_revision.txt'
                    detail = run.summary(time.time(), detail=True)
                    if revision.is_file():
                        detail['policy_revision'] = revision.read_text().strip()
                        counts = folder/'resume_status.json'
                        if counts.is_file():
                            status = json.loads(counts.read_text())
                            detail['rgb_resumptions'] = status.get('rgb_resumptions')
                            detail['rgb_activations'] = status.get('rgb_candidate_activations')
                    return self.send(200,detail)
            if url.path in ('/camera.jpg','/video.mp4','/api/media','/track-floor.jpg'):
                run=board.get(parse_qs(url.query).get('id',[''])[0])
                if not run: return self.send(404,{'error':'unknown run'})
                folder=(args.track_source if '-track-' in run.label else args.source)/run.label
                if url.path=='/track-floor.jpg':
                    file=folder/'run/simulation/board-floor.jpg'
                    if file.is_file():
                        return self.send(200,file.read_bytes(),'image/jpeg')
                    return self.send(404,{'error':'no track texture'})
                if url.path=='/api/media': return self.send(200,{'ready':(folder/'mission.mp4').is_file()})
                if url.path=='/video.mp4':
                    p=folder/'mission.mp4'
                    if not p.is_file(): return self.send(404,{'error':'pending'})
                    size=p.stat().st_size
                    raw=self.headers.get('Range')
                    first,last=0,size-1
                    if raw:
                        match=re.fullmatch(r'bytes=(\d*)-(\d*)',raw)
                        if not match or not any(match.groups()):
                            return self.send(416,b'',headers={'Content-Range':f'bytes */{size}'})
                        a,b=match.groups()
                        first=int(a) if a else max(0,size-int(b))
                        last=min(size-1,int(b)) if a and b else size-1
                        if first>last:
                            return self.send(416,b'',headers={'Content-Range':f'bytes */{size}'})
                    headers={'Accept-Ranges':'bytes'}
                    if raw: headers['Content-Range']=f'bytes {first}-{last}/{size}'
                    with p.open('rb') as stream:
                        stream.seek(first)
                        return self.send(206 if raw else 200,stream.read(last-first+1),'video/mp4',headers)
                files=sorted((folder/'camera').glob('*.jpg'))
                if files: return self.send(200,files[-2 if len(files)>1 else -1].read_bytes(),'image/jpeg')
                return self.send(404,{'error':'waiting for camera'})
            super().do_GET()
    ThreadingHTTPServer((args.host,8780),Handler).serve_forever()


if __name__=='__main__':main()
