'use strict';
const $ = id => document.getElementById(id);
const state = {data:null, campaign:'', job:'', detail:null, tab:'live', videoLimit:18, polling:false, jobsSignature:'', campaignSignature:'', videoSignature:'', imageKey:''};
const labels = {running:'실행 중',starting:'시작 준비',passed:'독립 검증 통과',failed:'미통과',invalid_environment:'환경 오류로 종료',stopped:'종료 / 중단',queued:'대기'};
const phases = {train:'학습',pilot:'파일럿',validation:'검증',final:'독립 시험',diagnostic:'환경 진단'};
const format = (n, digits=1) => typeof n === 'number' && Number.isFinite(n) ? n.toFixed(digits) : '—';
const esc = text => String(text ?? '').replace(/[&<>"']/g, ch => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[ch]));
const params = values => new URLSearchParams(values).toString();
const elapsed = n => `${Math.floor((n || 0)/60)}:${String(Math.floor((n || 0)%60)).padStart(2,'0')}`;
async function api(path) {
  const response = await fetch(path, {cache:'no-store'});
  if (!response.ok) throw new Error(`HTTP ${response.status}`);
  return response.json();
}
function setTab(tab) {
  if (!['live','videos','logs'].includes(tab)) tab='live';
  state.tab = tab;
  for (const name of ['live','videos','logs']) $(name+'-tab').hidden = tab !== name;
  document.querySelectorAll('.nav-item').forEach(button => button.classList.toggle('active',button.dataset.tab===tab));
  const titles = {live:['실시간 관제','로봇 관제실','로봇이 보는 화면부터 마커를 찾는 순간까지, 실제 실험의 흐름을 확인하세요.'],videos:['영상 보관함','실험 영상 보관함','완료된 주행을 다시 보고, 실험별 결과와 영상을 함께 확인하세요.'],logs:['실행 로그','실행 로그','선택한 실험의 ROS·Gazebo 로그와 마커 이벤트를 확인하세요.']};
  $('breadcrumb').textContent=titles[tab][0];
  $('page-title').innerHTML=esc(titles[tab][1])+'<span class="title-dot">.</span>';
  $('page-description').textContent=titles[tab][2];
  history.replaceState(null,'','#'+tab);
  if (tab==='logs') loadLogs();
  if (tab==='videos') renderArchive();
}
function setOptions(element, options, selected) {
  element.replaceChildren(...options.map(item => {
    const option=document.createElement('option'); option.value=item.value;option.textContent=item.label;return option;
  }));
  element.value=selected;
}
function renderSelectors(data) {
  if($('auto-follow').checked && !data.campaigns.find(c=>c.id===state.campaign)?.active && data.campaigns[0]?.active){state.campaign=data.campaigns[0].id;state.job='';}
  if (!data.campaigns.some(c=>c.id===state.campaign)) state.campaign=data.campaigns[0]?.id || '';
  const signature=data.campaigns.map(c=>c.id).join('|');
  if (signature!==state.campaignSignature) {
    setOptions($('campaign'),data.campaigns.map(c=>({value:c.id,label:c.id})),state.campaign);
    state.campaignSignature=signature;
  }
  $('campaign').value=state.campaign;
  const jobs=data.jobs.filter(j=>j.campaign===state.campaign).sort((a,b)=>a.number-b.number);
  if($('auto-follow').checked && !jobs.some(j=>j.id===state.job&&['running','starting'].includes(j.state)))state.job='';
  if (!jobs.some(j=>j.id===state.job)) state.job=(jobs.find(j=>['running','starting'].includes(j.state)) || jobs.find(j=>j.state==='passed') || jobs[0])?.id || '';
  const jobSig=jobs.map(j=>`${j.id}:${j.state}`).join('|');
  if(jobSig!==state.jobsSignature){
    setOptions($('job'),jobs.map(j=>({value:j.id,label:`#${String(j.number).padStart(3,'0')} · Seed ${j.seed} · ${j.obstacles ?? '?'} obstacles · ${labels[j.state] || j.state}`})),state.job);
    state.jobsSignature=jobSig;
  }
  $('job').value=state.job;
  const shapeLabels={pdf_original:'PDF 원래 공간',circle:'원형',triangle:'삼각형',trapezoid:'사다리꼴',rectangle:'직사각형',l_shape:'L자',u_shape:'U자',rooms:'다중 방'};
  const groups={};for(const job of jobs){const name=job.arena.shape;(groups[name] ||= []).push(job);}
  $('shape-summary').innerHTML=Object.entries(groups).map(([name,rows])=>`<span><b>${esc(shapeLabels[name] || name)}</b> ${rows.filter(j=>['passed','failed','invalid_environment'].includes(j.state)).length}/${rows.length} 종료 · ${rows.filter(j=>j.state==='passed').length} 통과</span>`).join('');
  const campaign=data.campaigns.find(c=>c.id===state.campaign);
  if(campaign){
    $('side-campaign').textContent=campaign.id;
    $('side-progress').textContent=`G${campaign.generation} · ${phases[campaign.phase] || campaign.phase}\n${campaign.completed} / ${campaign.total} 완료 · ${campaign.active}개 실행 중`;
    $('campaign-progress').style.width=`${100*campaign.completed/Math.max(1,campaign.total)}%`;
  }
}
function renderDetail(d) {
  state.detail=d;
  $('run-badge').textContent=labels[d.state] || d.state;
  $('run-badge').className='run-badge '+d.state;
  $('stat-targets').textContent=d.targets ?? '—';
  $('recognized-chips').innerHTML=d.confirmed_ids.map(id=>`<span class="chip">ID ${esc(id)}</span>`).join('') || '<span class="stat-note">아직 확인된 ID 없음</span>';
  $('stat-approached').textContent=d.audit?.actual_approached ?? d.approached_ids.length;
  $('approach-note').textContent=d.audit ? '종료 후 실제 주행 거리 독립 검증' : '주행 중 위치 추정 기준 · 최종 검증 전';
  $('stat-time').textContent=format(d.elapsed);
  $('time-note').textContent=`제한 600초 · RTF ${format(d.metrics.real_time_factor,2)}×`;
  $('stat-contacts').textContent=d.contacts ?? '—';
  $('contact-stat').classList.toggle('contact-warning', d.contacts>0);
  $('contact-note').textContent=d.contacts>0 ? '접촉 기록 확인 필요' : '기록된 접촉 0회';
  const active=['running','starting'].includes(d.state);
  $('camera-mode').textContent=d.camera.live ? 'LIVE' : active ? '저장 프레임' : 'LAST FRAME';
  $('camera-mode').classList.toggle('stale',!d.camera.live);
  $('camera-source').textContent=d.camera.live ? '실제 ROS RGB · 읽기 전용 구독' : '실제 녹화의 로봇 카메라 프레임';
  $('camera-age').textContent=d.camera.live ? `${format(d.camera.age,0)}초 전 수신` : d.frame_age!==null ? `${format(d.frame_age,0)}초 전 저장` : '프레임 대기';
  $('camera-stamp').textContent=d.camera.live ? `SIM ${format(d.camera.source_sim_seconds)}s` : `SEED ${d.seed}`;
  $('map-age').textContent=d.frame_age!==null ? `${format(d.frame_age,0)}초 전 저장` : '지도 대기';
  const shapes={pdf_original:'PDF 원래 공간',circle:'원형',triangle:'삼각형',trapezoid:'사다리꼴',rectangle:'직사각형',l_shape:'L자',u_shape:'U자',rooms:'다중 방'};
  $('scene-info').textContent=`${shapes[d.arena.shape] || d.arena.shape} · ${format(d.arena.width_m)}×${format(d.arena.height_m)}m · 장애물 ${d.obstacles ?? '?'}개`;
  $('domain-label').textContent=`DOMAIN ${d.domain ?? '—'}`;
  const stream=d.metrics.streams || {};
  const telemetry=[['현재 동작',active ? d.status.active_action || d.state : labels[d.state]],['공간 형태',shapes[d.arena.shape] || d.arena.shape],['실시간 배율',`${format(d.metrics.real_time_factor,2)}×`],['카메라',`${format(stream.camera?.sim_hz)} Hz (sim)`],['LiDAR',`${format(stream.scan?.sim_hz)} Hz (sim)`],['실행 서비스',d.service]];
  if(d.exploration_memory.path_distance_m!==undefined)telemetry.push(['주행 / 재방문 추정',`${format(d.exploration_memory.path_distance_m)} / ${format(d.exploration_memory.revisited_distance_m)}m`]);
  telemetry.push(['바닥 센서 검사',d.floor_sensor_gate===true?'4개 센서 통과':d.floor_sensor_gate===false?'실패 · 환경 재검증':'아직 검사 전']);
  telemetry.push(['정책·Nav2 적용 확인',d.runtime_configuration_gate===true?'실제 ROS 값 확인':d.runtime_configuration_gate===false?'확인 중 / 불일치':'검사 전']);
  $('telemetry').innerHTML=telemetry.map(([k,v])=>`<dt>${esc(k)}</dt><dd>${esc(v)}</dd>`).join('');
  $('readiness').textContent=d.status.readiness_reason || (active ? '센서·주행 상태 모니터링 중' : `종료 사유: ${d.reason || d.state}`);
  const events=d.events.filter(e=>e.event!=='mission_state').slice(-30).reverse();
  const eventLabels={id_confirmed:['◈','정확한 ID 인식','여러 관측의 일치 확인'],approach_estimated_within_1_5m:['◎','1.5m 접근 추정','최종 실제 거리는 종료 후 검증'],id_retracted:['↶','ID 확인 취소','인식 결과가 변경되었습니다'],approach_retracted:['↶','접근 확인 취소','접근 추정 기록이 변경되었습니다']};
  $('events').innerHTML=events.length ? events.map(e=>{const l=eventLabels[e.event] || ['·',e.event,''];return `<div class="event"><span class="event-icon ${e.event.includes('retracted')?'warn':''}">${l[0]}</span><div><strong>ID ${esc(e.id)} · ${esc(l[1])}</strong><p>${esc(l[2])}</p></div><time>${elapsed(e.elapsed_seconds)}</time></div>`;}).join('') : '<p class="empty-copy">아직 기록된 마커 이벤트가 없습니다.</p>';
  const auditPanel=$('audit-title').closest('.audit-panel');
  auditPanel.classList.toggle('passed',Boolean(d.audit?.passed));auditPanel.classList.toggle('failed',Boolean(d.audit&&!d.audit.passed));
  $('audit-title').textContent=d.audit ? d.audit.passed ? '독립 검증 통과 · 세 ID와 실제 접근을 모두 확인했습니다.' : '이 실행은 현재 성공 기준을 충족하지 못했습니다.' : '독립 검증은 실험 종료 후 표시됩니다.';
  $('audit-note').textContent=d.audit ? `${d.reason} · 좌표 허용 오차 0.5m · 실제 접근 반경 1.5m · 제한 600초` : '성공 기준: 정확한 ID 3개 · 실제 거리 ≤1.5m · 지도 오차 ≤0.5m · 접촉 0회 · 600초 이내';
  if(d.audit?.physical_credit_verified===false){$('audit-title').textContent='접근 완료 시점의 실제 거리 검증을 통과하지 못했습니다.';$('audit-note').textContent+=' · 이후 최근접 거리만으로 이전 완료 판정을 인정하지 않습니다.';}
  else if(d.audit?.physical_credit_verified===true){$('audit-note').textContent+=' · 완료 기록 시점 실제 거리도 검증했습니다.';}
  if(d.environment_invalidation.policy_training_sample_valid===false){$('audit-title').textContent='시뮬레이션 환경 오류 · 정책 학습 표본에서 제외했습니다.';$('audit-note').textContent=d.environment_invalidation.reason;}
  $('audit-details').innerHTML=d.audit ? Object.entries(d.audit.closest_approach_m).map(([id,distance])=>`<div class="audit-details-row">ID ${esc(id)} &nbsp; 실제 최근접 ${format(distance,2)}m &nbsp; 좌표 오차 ${format(d.audit.position_errors_m[id],3)}m</div>`).join('') : '';
  $('log-job').textContent=`${d.campaign} / #${d.number} / Seed ${d.seed}`;
  $('artifact-links').innerHTML=d.artifacts.map(name=>`<a href="/api/artifact?${params({id:d.id,name})}">${esc(name)} ↓</a>`).join('') || '<span class="subtle">실행 후 파일이 표시됩니다.</span>';
  $('open-composite').href='/api/frame?'+params({id:d.id,view:'composite'});
  if(state.imageKey!==d.id){state.imageKey=d.id;updateImages(true);}
  const warning=active && d.status_age!==null && d.status_age>45 ? `실시간 상태가 ${format(d.status_age,0)}초 동안 갱신되지 않았습니다. 서비스: ${d.service}. 로그를 확인하세요.` : state.data.observer_error;
  $('error-banner').textContent=warning || '';$('error-banner').hidden=!warning;
}
function updateImages(force=false) {
  if(!state.job || state.tab!=='live' || document.hidden) return;
  const views=[['robot-frame','robot'],['map-frame','map'],['overview-frame','overview']];
  for(const [id,view] of views){
    if(!force && view!=='robot' && Date.now() - Number($(id).dataset.updated || 0)<10000) continue;
    if(!force && $(id).dataset.loading==='1') continue;
    const image=$(id);image.dataset.updated=String(Date.now());image.dataset.loading='1';
    image.src='/api/frame?'+params({id:state.job,view,t:Date.now()});
  }
}
function videoCard(v){
  return `<article class="video-card"><button class="video-preview" data-video="${esc(v.id)}" aria-label="${esc(v.name)} 재생"><img src="${esc(v.thumbnail)}" loading="lazy" alt="실제 Gazebo 주행 녹화 미리보기"><span class="play-icon">▷</span><span class="video-duration">${elapsed(v.seconds)} SIM</span></button><div class="video-content"><h3 title="${esc(v.name)}">${esc(v.name)}</h3><div class="video-meta"><span class="video-result ${v.passed?'':'fail'}">${v.environment_valid===false?'환경 오류로 종료':v.passed?'✓ 현재 기준 통과':'미통과 / 검증 부족'}</span><span>ID ${v.targets ?? '—'}/3 · 접촉 ${v.contacts ?? '—'}</span></div></div></article>`;
}
function renderVideos(data){
  $('video-count').textContent=data.videos.length;
  const sig=data.videos.map(v=>`${v.id}:${v.passed}:${v.environment_valid}`).join('|');
  if(sig!==state.videoSignature){
    $('recent-videos').innerHTML=data.videos.slice(0,3).map(videoCard).join('') || '<p class="empty-copy">완료 영상을 기다립니다.</p>';
    $('recent-count').textContent=`${data.videos.length}개 보관 중`;
    state.videoSignature=sig;
    renderArchive();
  }
}
function renderArchive(){
  if(!state.data) return;
  const search=$('video-search').value.toLowerCase();const filter=$('video-filter').value;
  const videos=state.data.videos.filter(v=>v.name.toLowerCase().includes(search)&&(filter==='all'||(filter==='passed')===v.passed));
  $('archive-count').textContent=`${videos.length}개 영상 · MP4 · 시뮬레이션 1× 재생 · 과거 성공 표기와 현재 1.5m 검증 기준은 다를 수 있습니다.`;
  $('archive-videos').innerHTML=videos.slice(0,state.videoLimit).map(videoCard).join('') || '<p class="empty-copy">조건에 맞는 영상이 없습니다.</p>';
  $('more-videos').hidden=videos.length<=state.videoLimit;
}
function playVideo(id){
  const video=state.data?.videos.find(v=>v.id===id);if(!video)return;
  $('player-title').textContent=video.name;
  $('player').src=video.url;
  $('player-info').textContent=`${format(video.seconds)} sim s · ${(video.bytes/1048576).toFixed(1)} MB · ${video.environment_valid===false?'환경 오류로 종료':video.passed?'현재 기준 통과':'미통과 / 검증 부족'}`;
  $('download-video').href=video.url+'&download=1';
  $('video-dialog').showModal();
}
async function loadLogs(){
  if(!state.job)return;
  const selected=state.job;
  try{
    const data=await api('/api/log?'+params({id:selected,source:$('log-source').value}));
    if(selected!==state.job)return;
    const output=$('log-output');output.textContent=data.text || '아직 기록된 로그가 없습니다.';
    if($('log-follow').checked)output.scrollTop=output.scrollHeight;
  }catch(error){$('log-output').textContent=`로그 조회 실패: ${error.message}`;}
}
async function poll(){
  if(state.polling)return;state.polling=true;
  try{
    const data=await api('/api/state');state.data=data;renderSelectors(data);renderVideos(data);
    if(state.job){const selected=state.job;const detail=await api('/api/job?'+params({id:selected}));if(selected===state.job)renderDetail(detail);}
    $('connection').textContent='로컬 서버 연결됨';$('connection-dot').classList.remove('offline');
    $('refresh-label').textContent='3초마다 상태 갱신';
    if(state.tab==='logs')await loadLogs();
  }catch(error){
    $('connection').textContent='서버 연결 끊김';$('connection-dot').classList.add('offline');
    $('error-banner').hidden=false;$('error-banner').textContent=`서버 응답이 없습니다 (${error.message}). 화면은 마지막 수신 데이터입니다.`;
    $('camera-mode').textContent='연결 끊김';$('camera-mode').classList.add('stale');
  }finally{state.polling=false;}
}
document.querySelectorAll('[data-tab]').forEach(b=>b.addEventListener('click',()=>setTab(b.dataset.tab)));
document.querySelector('.brand').addEventListener('click',()=>setTab('live'));
$('campaign').addEventListener('change',()=>{$('auto-follow').checked=false;state.campaign=$('campaign').value;state.job='';state.jobsSignature='';poll();});
$('job').addEventListener('change',()=>{$('auto-follow').checked=false;state.job=$('job').value;poll();});
$('auto-follow').addEventListener('change',poll);
$('refresh').addEventListener('click',poll);
$('log-source').addEventListener('change',loadLogs);
$('video-search').addEventListener('input',()=>{state.videoLimit=18;renderArchive();});
$('video-filter').addEventListener('change',()=>{state.videoLimit=18;renderArchive();});
$('more-videos').addEventListener('click',()=>{state.videoLimit+=18;renderArchive();});
document.addEventListener('click',event=>{const target=event.target.closest('[data-video]');if(target)playVideo(target.dataset.video);});
$('close-player').addEventListener('click',()=>$('video-dialog').close());
$('video-dialog').addEventListener('close',()=>{$('player').pause();$('player').removeAttribute('src');$('player').load();});
$('fullscreen').addEventListener('click',()=>{$('camera-stage').requestFullscreen?.();});
$('copy-log').addEventListener('click',async()=>{try{await navigator.clipboard.writeText($('log-output').textContent);$('copy-log').textContent='복사됨';setTimeout(()=>$('copy-log').textContent='복사',1500);}catch{ $('copy-log').textContent='로그 선택 후 복사'; }});
$('download-log').addEventListener('click',()=>{const blob=new Blob([$('log-output').textContent],{type:'text/plain;charset=utf-8'});const url=URL.createObjectURL(blob);const link=document.createElement('a');link.href=url;link.download=`${state.detail?.seed || 'robot'}-${$('log-source').value}.log`;link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});
for(const id of ['robot-frame','map-frame','overview-frame']){
  $(id).addEventListener('load',()=>{$(id).classList.remove('broken');$(id).dataset.loading='0';$(id).nextElementSibling.hidden=true;});
  $(id).addEventListener('error',()=>{$(id).classList.add('broken');$(id).dataset.loading='0';$(id).nextElementSibling.hidden=false;});
}
setInterval(()=>$('wall-clock').textContent=new Date().toLocaleString('ko-KR',{month:'2-digit',day:'2-digit',hour:'2-digit',minute:'2-digit',second:'2-digit',hour12:false}),1000);
setInterval(poll,3000);setInterval(updateImages,1500);
document.addEventListener('visibilitychange',()=>{if(!document.hidden){poll();updateImages(true);}});
setTab(location.hash.slice(1));poll();
