"""Publish completed local Gazebo recordings as immutable, named release assets.

Run from this checkout with an authenticated GitHub CLI. The remote release must
already exist. Does not commit, push source, overwrite assets, or publish runs
without final evaluation and finalized recording metadata.
"""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import cv2


def command(args):
    return subprocess.check_output(args,text=True)


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--repo',default='peter010905/simul')
    parser.add_argument('--tag',default='simulation-videos-2026-11')
    parser.add_argument('--upload',action='store_true')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    gh=shutil.which('gh') or str(Path.home()/'.local/bin/gh')
    base=root/'training_robust/native90'
    release=json.loads(command([gh,'api',f'repos/{args.repo}/releases/tags/{args.tag}']))
    def assets():
        pages=json.loads(command([gh,'api','--paginate','--slurp',f'repos/{args.repo}/releases/{release["id"]}/assets?per_page=100']))
        return {a['name']:a for page in pages for a in page}
    remote=assets();entries=[];skipped=[]
    videos=sorted(base.rglob('video/mission.mp4'),key=lambda p:('fix_v' not in str(p),str(p)))
    staging=root/'tmp';staging.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='release-videos-',dir=staging) as temp:
        for video in videos:
            episode=video.parent.parent
            evaluation=episode/'evaluation.json';recording=video.parent/'recording.json'
            if not evaluation.exists() or not recording.exists():
                skipped.append(dict(path=str(video.relative_to(root)),reason='Final evaluation or recording metadata absent'));continue
            result=json.loads(evaluation.read_text());rec=json.loads(recording.read_text())
            if rec.get('error') or rec.get('frames',0)<=0:
                skipped.append(dict(path=str(video.relative_to(root)),reason='Recording error or empty video'));continue
            capture=cv2.VideoCapture(str(video))
            frame_count=int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
            first_ok,_=capture.read()
            capture.set(cv2.CAP_PROP_POS_FRAMES,max(0,frame_count-1))
            last_ok,_=capture.read();capture.release()
            if not first_ok or not last_ok or frame_count!=rec['frames']:
                skipped.append(dict(path=str(video.relative_to(root)),reason='Final video frame count not verified'));continue
            parts=episode.relative_to(base).parts
            name='__'.join(p for p in parts if p not in ('jobs','episodes'))+'.mp4'
            sha=digest(video);size=video.stat().st_size
            if name in remote:
                item=remote[name]
                if item['size']!=size or item.get('digest')!='sha256:'+sha:
                    raise RuntimeError('Remote asset differs or lacks hash verification; preserving it: '+name)
            elif args.upload:
                target=Path(temp)/name
                os.link(video,target)
                subprocess.run([gh,'release','upload',args.tag,str(target),'--repo',args.repo],check=True,capture_output=True,text=True)
                target.unlink()
                print('Uploaded '+name,flush=True)
            else:
                print('Pending '+name,flush=True)
            audit=result.get('ground_truth_audit',{})
            distances=audit.get('closest_approach_m',{})
            approach=bool(len(distances)==3 and all(v is not None and v<=1.5 for v in distances.values()))
            entries.append(dict(asset=name,local_path=str(video.relative_to(root)),bytes=size,sha256=sha,
                url=f'https://github.com/{args.repo}/releases/download/{args.tag}/{name}',
                original_evaluator_success=result.get('success'),reason=result.get('reason'),
                elapsed_sim_seconds=result.get('elapsed_seconds'),targets=result.get('targets'),collisions=result.get('collisions'),
                actual_approach_1_5m=approach if distances else None,
                recording_frames=rec['frames'],recording_seconds=rec.get('duration_seconds')))
    remote=assets()
    for item in entries:
        a=remote.get(item['asset'])
        item['uploaded_verified']=bool(a and a['size']==item['bytes'] and a.get('digest')=='sha256:'+item['sha256'])
    report=dict(updated_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),release_url=release['html_url'],
        uploaded_verified=sum(e['uploaded_verified'] for e in entries),videos=entries,skipped=skipped,
        note='Original evaluator success may use an older protocol; actual 1.5m approach is listed separately. Login required for private release downloads.')
    path=root/'repository_snapshot/videos.json';path.write_text(json.dumps(report,indent=2)+'\n')
    lines=['<!doctype html><html lang="ko"><meta charset="utf-8"><title>Gazebo videos</title>',
           '<style>body{font:15px sans-serif;max-width:1200px;margin:32px auto}td,th{padding:8px;border-bottom:1px solid #ddd;text-align:left}a{color:#1763ad}</style>',
           '<h1>Gazebo 영상 목록</h1><p>비공개 저장소 권한이 있는 GitHub 계정으로 로그인해야 다운로드할 수 있습니다. 성공 여부는 당시 평가 기준입니다. 실제 1.5m 접근 여부를 별도로 확인하세요.</p>',
           '<table><tr><th>영상</th><th>당시 평가</th><th>실제 1.5m 접근</th><th>시뮬레이션 시간</th></tr>']
    for e in entries:
        if not e['uploaded_verified']:continue
        approach='확인' if e['actual_approach_1_5m'] else '미달' if e['actual_approach_1_5m'] is False else '자료 없음'
        lines.append(f'<tr><td><a href="{e["url"]}">{e["asset"]}</a></td><td>{"성공" if e["original_evaluator_success"] else "실패"}</td><td>{approach}</td><td>{e["elapsed_sim_seconds"]}</td></tr>')
    lines.append('</table></html>');(root/'repository_snapshot/videos.html').write_text('\n'.join(lines)+'\n')
    start='<!-- SIMULATION_VIDEO_INDEX:START -->'
    end='<!-- SIMULATION_VIDEO_INDEX:END -->'
    table=[start,'','## 저장된 Gazebo 영상','',
           f'[GitHub Release에서 다운로드]({release["html_url"]}) · 검증된 영상 {report["uploaded_verified"]}개','',
           '아래 성공 여부는 당시 평가 결과이며, 현재 기준의 독립 완주율을 뜻하지 않습니다.','',
           '<details><summary>전체 영상과 평가 결과 펼치기</summary>','',
           '| 영상 | 당시 평가 | 실제 1.5m 접근 | 시간(시뮬레이션 초) |',
           '|---|---|---|---:|']
    for e in entries:
        if not e['uploaded_verified']:continue
        approach='확인' if e['actual_approach_1_5m'] else '미달' if e['actual_approach_1_5m'] is False else '자료 없음'
        table.append(f'| [{e["asset"]}]({e["url"]}) | {"성공" if e["original_evaluator_success"] else "실패"} | {approach} | {e["elapsed_sim_seconds"]} |')
    table.extend(['','</details>','',end])
    readme=root/'README.md';content=readme.read_text();block='\n'.join(table)
    if start in content and end in content:
        a=content.index(start);b=content.index(end,a)+len(end);content=content[:a]+block+content[b:]
    else:content+='\n'+block+'\n'
    readme.write_text(content)
    print(json.dumps(dict(uploaded_verified=report['uploaded_verified'],eligible=len(entries),skipped=len(skipped))),flush=True)
    if args.upload and report['uploaded_verified']!=len(entries):raise RuntimeError('Not all videos were verified remotely')


if __name__=='__main__':main()
