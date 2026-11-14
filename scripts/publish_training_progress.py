"""Publish experiment summaries/videos only, in an isolated checkout.

This does not edit policy code, review code, promote a policy or control a robot.
The local timer runs only while this host's user services are available.
"""
import argparse
import collections
import datetime
import fcntl
import json
from pathlib import Path
import subprocess
import sys

START='<!-- ROBOT_TRANSFER_PROGRESS:START -->'
END='<!-- ROBOT_TRANSFER_PROGRESS:END -->'
FILES=['README.md','repository_snapshot/status.json','repository_snapshot/robot_transfer_progress.json',
       'repository_snapshot/videos.json','repository_snapshot/videos.html']


def run(args, cwd=None, timeout=120):
    return subprocess.run(args,cwd=cwd,text=True,capture_output=True,check=True,timeout=timeout).stdout


def summarize(campaign):
    state=json.loads((campaign/'state.json').read_text())
    done=[j for j in state['jobs'] if j['status']=='done']
    valid=[j for j in done if not j['outcome']['infrastructure_failure']]
    by_group={}
    for job in valid:
        key='/'.join([job['phase'],job['role'],job.get('stress_profile','historical'),job['scenario']['arena']['shape']])
        row=by_group.setdefault(key,dict(completed=0,successes=0,contacts=0))
        row['completed']+=1;row['successes']+=int(job['outcome']['success']);row['contacts']+=job['outcome']['contacts']
    active=[]
    for j in state['jobs']:
        if j['status']!='running':continue
        result=subprocess.run(['systemctl','--user','is-active',j['unit']],capture_output=True,text=True,timeout=5)
        row=dict(number=j['number'],unit=j['unit'],service_active=result.returncode==0,
                 shape=j['scenario']['arena']['shape'],profile=j.get('stress_profile'),
                 gates={k:v for k,v in j.items() if k.endswith('_gate')})
        p=Path(j['output'])/'episodes'/str(j['scenario']['seed'])/'live_status.json'
        if p.exists():
            live=json.loads(p.read_text()).get('status',{})
            row['live']={k:live.get(k) for k in ('state','targets','collisions','elapsed_seconds','readiness_reason')}
        active.append(row)
    coordinator=subprocess.run(['systemctl','--user','is-active',state['coordinator_unit']],capture_output=True,text=True,timeout=5)
    return dict(campaign=campaign.name,status=state['status'],coordinator_active=coordinator.returncode==0,
        generation=state['generation'],phase=state['phase'],recorded_completed=len(done),
        valid_completed=len(valid),successes=sum(j['outcome']['success'] for j in valid),
        infrastructure_failures=len(done)-len(valid),by_phase_role_profile_shape=by_group,active=active,
        evidence_type='native_gazebo_training_and_paired_development_not_hardware',
        production_promoted=False,hardware_validated=False)


def snapshot(source, checkout, campaign):
    summary=summarize(campaign)
    summary['captured_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat()
    summary['synthetic_stress_is_real_robot_calibration']=False
    summary['automation_scope']='local systemd timer: summary and finished-video publication only'
    path=checkout/'repository_snapshot/robot_transfer_progress.json'
    path.write_text(json.dumps(summary,indent=2)+'\n')
    status_path=checkout/'repository_snapshot/status.json'
    status=json.loads(status_path.read_text())
    status['robot_transfer_progress']='repository_snapshot/robot_transfer_progress.json'
    status['robot_transfer_training_observation']=summary
    status_path.write_text(json.dumps(status,indent=2)+'\n')
    readme=checkout/'README.md';text=readme.read_text()
    block='\n'.join([START,'',
        f"실물 대응 후보 `{summary['campaign']}`: 상태 `{summary['status']}`, 세대 {summary['generation']}, 단계 `{summary['phase']}`.",
        f"독립 입력 검사를 통과한 native 개발 회차 {summary['valid_completed']}개 중 엄격한 완주 {summary['successes']}개; 인프라 실패 {summary['infrastructure_failures']}개 별도.",
        '형태·잡음·후보/기준별 결과는 [진행 기록](repository_snapshot/robot_transfer_progress.json)에 분리합니다. 학습 중 관측이며 실물 검증·production 승격 결과가 아닙니다.',
        f"관측 UTC: {summary['captured_utc']}. 로컬 게시 작업은 요약·완성 영상만 갱신하며 자동 코드 수정/리뷰를 수행하지 않습니다.",'',END])
    if START in text and END in text:
        a=text.index(START);b=text.index(END,a)+len(END);text=text[:a]+block+text[b:]
    else:text+='\n'+block+'\n'
    readme.write_text(text)
    return summary


def publish(source, checkout, campaign, videos):
    checkout.parent.mkdir(parents=True,exist_ok=True)
    if not (checkout/'.git').exists():
        remote=run(['git','remote','get-url','origin'],source).strip()
        run(['git','clone','--quiet','--branch','main','--single-branch',remote,str(checkout)],timeout=180)
    # Only our fixed generated-file list may be dirty from an interrupted run.
    changed=run(['git','diff','--name-only','HEAD'],checkout).splitlines()
    if set(changed)-set(FILES):raise RuntimeError('Unexpected edits in publication checkout')
    if changed:run(['git','restore','--source=HEAD','--staged','--worktree','--',*changed],checkout)
    run(['git','fetch','origin','main'],checkout)
    run(['git','merge','--ff-only','origin/main'],checkout)
    if videos:
        run([sys.executable,str(checkout/'scripts/publish_simulation_videos.py'),
             '--data-root',str(source),'--upload'],cwd=checkout,timeout=1800)
    snapshot(source,checkout,campaign)
    run(['git','add','--',*FILES],checkout)
    if not run(['git','diff','--cached','--name-only'],checkout).strip():return
    for field in ('user.name','user.email'):
        run(['git','config',field,run(['git','config',field],source).strip()],checkout)
    run(['git','commit','-m','Update native robot-transfer progress and finalized video index'],checkout)
    # Normal push rejects concurrent main changes. Reconcile on the next tick;
    # never force-push or alter the user's working checkout.
    run(['git','push','origin','HEAD:main'],checkout,timeout=180)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,default=Path(__file__).resolve().parents[1])
    p.add_argument('--campaign',type=Path,required=True)
    p.add_argument('--checkout',type=Path)
    p.add_argument('--publish',action='store_true');p.add_argument('--videos',action='store_true')
    a=p.parse_args();source=a.source.resolve();campaign=a.campaign.resolve()
    lock_path=source/'deployment/training_progress.lock';lock_path.parent.mkdir(exist_ok=True)
    with lock_path.open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if a.publish:
            if a.checkout is None:p.error('--publish requires an isolated --checkout')
            if a.checkout.resolve()==source:p.error('Publication checkout must be isolated')
            publish(source,a.checkout.resolve(),campaign,a.videos)
        else:print(json.dumps(snapshot(source,source,campaign)))
