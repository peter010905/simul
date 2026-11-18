const $=id=>document.getElementById(id);
const sec=(value)=>value===undefined||value===null?'수신 전':`${Number(value).toFixed(1)}초 전`;
const oldest=(...values)=>{const known=values.filter(v=>v!==undefined&&v!==null);return known.length?Math.max(...known):undefined};
const number=(value,digits=2)=>value===null||value===undefined?'—':Number(value).toFixed(digits);
let controlToken='',commandSequence=0;
const pendingCommands=new Set();
function renderControl(control){
  const enabled=Boolean(control?.enabled);
  $('robot-controls').hidden=!enabled;
  if(!enabled){controlToken='';return}
  controlToken=control.token||'';
  const labels={idle:'구동 대기',queued:'구동 요청 접수',preflight:'시간·센서 확인 중',starting:'주행 준비 중',running:'주행 중',stopping:'정지 중',stopped:'정지 완료',failed:'구동 확인 필요',stop_unverified:'정지 확인 필요'};
  $('control-state').textContent=labels[control.phase]||control.phase;
  $('control-state').className=['failed','stop_unverified'].includes(control.phase)?'bad':control.phase==='running'?'good':'';
  $('control-detail').textContent=control.detail||'';
  $('robot-start-r32').disabled=$('robot-start').disabled=!controlToken||!control.can_start||pendingCommands.size>0;
  $('robot-stop').disabled=!controlToken||pendingCommands.has('stop');
  $('robot-start').textContent=['queued','preflight','starting'].includes(control.phase)?'준비 중…':'기존 정책 구동';
  const started=(control.events||[]).find(e=>e.event==='mission_started');
  $('control-timing').textContent=started?`자동 시작까지 ${number(started.elapsed_s,1)}초`:'';
}
async function controlCommand(action){
  if(!controlToken||pendingCommands.has(action))return;
  const sequence=++commandSequence;
  pendingCommands.add(action);
  $('robot-start-r32').disabled=$('robot-start').disabled=true;
  $('robot-stop').disabled=pendingCommands.has('stop');
  $('control-detail').textContent=action!=='stop'?'구동 요청을 보내는 중…':'정지 요청을 보내는 중…';
  try{
    const response=await fetch(`/api/control/${action}`,{method:'POST',headers:{'Content-Type':'application/json','X-OSP-Control-Token':controlToken},body:'{}'});
    const result=await response.json();
    if(!response.ok)throw new Error(result.error||`HTTP ${response.status}`);
    if(sequence===commandSequence)renderControl(result);
  }catch(error){
    if(sequence===commandSequence){$('control-state').textContent='요청 확인 필요';$('control-detail').textContent=error.message}
  }finally{
    pendingCommands.delete(action);
    $('robot-stop').disabled=pendingCommands.has('stop');
    poll();
  }
}
function rows(id,items){
  const parent=$(id);parent.replaceChildren();
  for(const [name,value,kind] of items){
    const dt=document.createElement('dt'),dd=document.createElement('dd');
    dt.textContent=name;dd.textContent=String(value??'—');if(kind)dd.className=kind;
    parent.append(dt,dd);
  }
}
function image(id,empty,ready,age,limit,path){
  const img=$(id),placeholder=$(empty);
  if(ready && age!==undefined){
    img.src=`${path}?t=${Date.now()}`;placeholder.hidden=true;
    img.onerror=()=>{placeholder.hidden=false;img.removeAttribute('src')};
  }else{placeholder.hidden=false;img.removeAttribute('src')}
  const label=id==='map-image'?$('map-age'):id==='camera-image'?$('camera-age'):null;
  if(label){label.textContent=sec(age);label.classList.toggle('stale',age===undefined||age>limit)}
}
function render(data){
  const ages=data.ages_s||{},status=data.status||{},scan=data.scan||{},cell=data.start_cell||{};
  const sim=data.mode==='simulation';
  const statusLimit=sim?10:3,visualLimit=sim?15:5;
  $('board-title').textContent=sim?'Gazebo 시뮬 주행 보드':'실물 주행 보드';
  $('board-subtitle').textContent=`${sim?'ROS 시뮬 시간':'실물 ROS'} · ${data.control?.enabled?'보드에서 직접 구동·정지':'읽기 전용'}`;
  const dockBlocksStart=data.docked===true&&data.control?.can_start;
  renderControl(dockBlocksStart?{...data.control,can_start:false,detail:'로봇이 도크에 있어요. 바닥으로 옮기면 구동할 수 있어요.'}:data.control);
  const live=(ages.mission_status!==undefined&&ages.mission_status<statusLimit)||(ages.raw_scan!==undefined&&ages.raw_scan<5);
  $('connection').textContent=live?'ROS 수신 중':sim&&ages.raw_scan!==undefined?'시뮬 종료 · 마지막 화면':sim?'시뮬 시작 대기':'ROS 데이터 대기 / 끊김';
  $('connection').classList.toggle('offline',!live);
  const outcome=data.mission_display||{label:status.state==='done'?'종료 · 미완료':status.state||'상태 대기',detail:'',kind:''};
  $('mission-state').textContent=!live&&sim&&ages.raw_scan!==undefined?'시뮬 종료':outcome.label;
  $('mission-state').className=ages.mission_status>statusLimit?'bad':outcome.kind;
  $('mission-detail').textContent=`${outcome.detail||status.active_action||'동작 없음'} · 상태 ${sec(ages.mission_status)}`;
  const cellName={start_free:'빈 공간',start_not_free:'출발 셀 점유',start_unknown:'출발 셀 미관측',start_outside_map:'지도 밖',map_or_pose_unavailable:'지도 / TF 대기'};
  $('start-cell').textContent=cellName[cell.reason]||cell.reason||'대기';
  $('start-cell').className=cell.reason==='start_not_free'?'bad':cell.reason==='start_free'?'good':'';
  $('start-cell-detail').textContent=cell.value===undefined?`지도 ${sec(ages.map)} · TF 확인 중`:`지도 값 ${cell.value} · 셀 (${cell.x}, ${cell.y})`;
  const collision=data.collision_monitor||{};
  $('collision').textContent=collision.action||'상태 대기';
  $('collision').className=collision.action==='STOP'?'bad':'';
  $('collision-detail').textContent=`${collision.polygon||'다각형 미수신'} · ${sec(ages.collision_monitor)}`;
  const ids=status.approached_ids||[];
  const detected=data.discovered_ids||[];
  $('detected-markers').textContent=`${detected.length} / 3`;
  $('detected-detail').textContent=detected.length?`탐지 확인 ID ${detected.join(', ')}`:'탐지 확인된 ID 없음';
  $('markers').textContent=`${ids.length} / 3`;
  $('marker-detail').textContent=ids.length?`접근 확인 ID ${ids.join(', ')}`:'접근 확인된 ID 없음';
  const found=data.latest_discovery;
  const alert=$('marker-alert');
  if(found&&data.server_time-found.time<15){
    alert.textContent=`마커 발견: ID ${found.ids.join(', ')} · 현재 ${detected.length} / 3개 탐지`;
    alert.hidden=false;
  }else alert.hidden=true;
  rows('scan-details',[
    ['원본 LiDAR',sec(ages.raw_scan),ages.raw_scan>3?'bad':''],
    ['필터 LiDAR',sim?'원본 스캔 사용':sec(ages.filtered_scan),!sim&&ages.filtered_scan>3?'bad':''],
    ['원본 최소 거리',scan.min_range_m===null?'반사 없음':`${number(scan.min_range_m,3)} m`],
    ['LiDAR 기준 25cm 미만',scan.within_25cm_from_laser??'—'],
    ['몸체 내부 제거',sim?'시뮬 후보 미적용':scan.filtered_frame_matched?`${scan.removed_returns}개`:'동일 시각 프레임 대기',scan.removed_returns>0?'warn':'']
  ]);
  const pose=data.robot_pose;
  rows('nav-details',[
    ['현재 동작',status.active_action||'없음'],
    ...(status.drive_policy ? [['R32 주행 판단',({detour:'짧은 우회',frontier:'미지 경계로 이동',inspect:'관측 위치로 이동',no_route:'안전한 경로 대기',waiting_for_global_costmap:'비용 지도 대기'})[status.drive_policy.kind||status.drive_policy.reason]||status.drive_policy.kind||'대기']] : []),
    ...(status.navigation_flow?.candidates?.proposed!==undefined ? [['기존 정책 후보',`${status.navigation_flow.candidates.proposed}개 중 ${status.navigation_flow.candidates.selected_from}개${status.navigation_flow.candidates.shared_exit?' · 같은 출구의 다른 목표 허용':''}`]] : []),
    ['종료 사유',outcome.detail||'진행 중 / 대기'],
    ['미션 경과',status.elapsed_seconds===undefined?'—':`${number(status.elapsed_seconds,1)} s`],
    ['시작 준비',status.readiness_reason||'별도 거절 사유 없음',status.readiness_reason?'warn':''],
    ['로봇 위치',pose?`${number(pose[0])}, ${number(pose[1])} m`:'TF 대기'],
    ['전역 / 로컬 계획',`${data.plans?.global||0} / ${data.plans?.local||0}`],
    ['접촉 감지',status.collisions??'—',status.collisions>0?'bad':''],
    ['센서 중단',status.sensor_interruptions??'—']
  ]);
  rows('ages',[
    ['지도 / 그림',sec(oldest(ages.map,ages.map_image)),oldest(ages.map,ages.map_image)>visualLimit?'bad':''],
    ['로봇 TF',sec(data.pose_age_s),data.pose_age_s>3?'bad':''],
    ['카메라',sec(ages.camera),ages.camera>5?'bad':''],
    ['미션 상태',sec(ages.mission_status),ages.mission_status>statusLimit?'bad':''],
    ['충돌 감시',sec(ages.collision_monitor),ages.collision_monitor>3?'bad':'']
  ]);
  image('map-image','map-empty',data.map_ready,oldest(ages.map,ages.map_image),visualLimit,'/api/map.png');
  image('camera-image','camera-empty',data.camera_ready,ages.camera,visualLimit,'/api/camera.jpg');
  image('scan-image','scan-empty',data.scan_ready,oldest(ages.raw_scan,ages.scan_image),visualLimit,'/api/scan.png');
  $('source').textContent=`osp_mission: ${data.policy_prefix||'설치 경로 확인 불가'} · ${data.control?.enabled?'직접 제어 사용':'읽기 전용'}`;
  $('domain').textContent=`DOMAIN ${data.ros_domain_id} · ${data.namespace||'루트 네임스페이스'}`;
  const warnings=[];
  if(cell.reason==='start_not_free')warnings.push('출발 위치의 지도 셀이 점유 상태입니다. 원본/필터 LiDAR와 지도상의 로봇 위치를 대조하세요.');
  if(collision.action==='STOP')warnings.push('Nav2 충돌 감시가 STOP 상태입니다.');
  if(!live)warnings.push(sim&&ages.raw_scan!==undefined?'Gazebo 실행이 끝났습니다. 현재 화면은 마지막으로 받은 데이터입니다.':
    'ROS 데이터가 갱신되지 않습니다. 도메인·네임스페이스·RMW와 로봇 bringup을 확인하세요.');
  if(live&&ages.map!==undefined&&ages.map>visualLimit)warnings.push('지도가 갱신되지 않습니다. 표시된 경로와 출발 셀은 오래된 지도 기준입니다.');
  if(live&&data.pose_age_s!==null&&data.pose_age_s>3)warnings.push('map→base_link TF가 3초 넘게 갱신되지 않아 출발 셀 판정을 숨겼습니다.');
  if(!sim&&ages.filtered_scan===undefined&&ages.raw_scan!==undefined&&ages.raw_scan<3)warnings.push('필터 스캔이 없습니다. v21 필터가 실행 중인지 확인하세요.');
  $('warning').textContent=warnings.join(' ');$('warning').hidden=!warnings.length;
}
async function poll(){
  try{
    const response=await fetch('/api/state',{cache:'no-store'});
    if(!response.ok)throw new Error(`HTTP ${response.status}`);
    render(await response.json());
  }catch(error){
    $('robot-start-r32').disabled=$('robot-start').disabled=true;
    $('connection').textContent='보드 서버 연결 끊김';$('connection').classList.add('offline');
    $('warning').textContent=`보드 서버 응답 없음: ${error.message}`;$('warning').hidden=false;
  }
}
$('map-fullscreen').addEventListener('click',()=>$('map-frame').requestFullscreen?.());
$('robot-start-r32').addEventListener('click',()=>controlCommand('start-r32'));
$('robot-start').addEventListener('click',()=>controlCommand('start'));
$('robot-stop').addEventListener('click',()=>controlCommand('stop'));
poll();setInterval(poll,1000);
