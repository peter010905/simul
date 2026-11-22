let archiveView = new URLSearchParams(location.search).get('view') === 'archive';
let versionFilter = new URLSearchParams(location.search).get('version') || '';
let searchText = new URLSearchParams(location.search).get('q') || '';
let catalog = null;
let loadingView = null;
let refreshAgain = false;
let historyKey = '';
const openedGroups = new Set();
const limits = new Map();
const descriptions = new Map();
const displayPhases = {...PHASE, unstarted: '미출발'};
const dates = new Intl.DateTimeFormat('ko-KR', {month:'numeric',day:'numeric',hour:'2-digit',minute:'2-digit',hour12:false});

function runName(row) { return row.seed == null ? row.label : '맵 '+row.seed; }
function recordDate(row) { return row.started ? dates.format(new Date(row.started*1000)) : ''; }
function fixtureName(row) { return row.boxes==null?'':'추가 상자 '+row.boxes+'개 · 마커 '+(row.total ?? '?')+'개'; }
function filtersMatch(row) {
  const needle = searchText.trim().toLocaleLowerCase();
  return (!versionFilter || row.version===versionFilter) &&
    (!needle || [row.version,row.label,row.seed,row.id].join(' ').toLocaleLowerCase().includes(needle));
}
function rememberFilters() {
  const url = new URL(location.href);
  for (const [key,value] of [['view',archiveView?'archive':''],['version',versionFilter],['q',searchText]]) {
    if (value) url.searchParams.set(key,value); else url.searchParams.delete(key);
  }
  history.replaceState(null,'',url);
}
function historyLine(row) {
  const wasArchived=archiveView;
  const line = el('div',{class:'history-line'});
  const open = el('button',{type:'button',class:'history-open',title:row.label});
  const left = el('span');
  left.append(el('strong',{},runName(row)),el('small',{},[recordDate(row),fixtureName(row)].filter(Boolean).join(' · ')));
  let verdict = displayPhases[row.phase] || row.phase;
  if (row.strict===true) verdict='완주';
  else if (row.strict===false && row.phase==='finished') verdict='일부 미달';
  const proof = row.total==null ? String(row.approached) : row.approached+'/'+row.total;
  open.append(left,el('span',{class:'history-metrics'},verdict+' · 접근 '+proof+' · '+clock(row.sim_time)));
  open.addEventListener('click',()=>openRun(row.id));
  const action = el('button',{type:'button',class:'archive-action',
    'aria-label':runName(row)+(archiveView?' 실험으로 복원':' 보관함으로 이동')},archiveView?'복원':'보관');
  action.addEventListener('click',async ()=>{
    action.disabled=true;
    const toArchive=!wasArchived;
    try {
      const res=await fetch('/api/experiment-visibility',{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({id:row.id,archived:toArchive})});
      const reply=await res.json();
      if (!res.ok) throw new Error(reply.error || '변경을 저장할 수 없습니다.');
      $('libraryStatus').textContent=runName(row)+(toArchive?' 주행을 보관했습니다.':' 주행을 복원했습니다.');
      await poll();
    } catch(e) { $('libraryStatus').textContent=e.message; action.disabled=false; }
  });
  line.append(open,action);
  return line;
}
function groupNode(version,rows) {
  const key=(archiveView?'archive/':'current/')+version;
  const node=el('details',{class:'version-group'});
  const summary=el('summary');
  summary.append(el('strong',{},version),el('span',{},rows.length+'회'));
  const body=el('div');
  node.append(summary,body);
  function fill() {
    const count=limits.get(key)||8;
    const items=rows.slice(0,count).map(historyLine);
    if (rows.length>count) {
      const more=el('button',{type:'button',class:'more-runs'},'이전 주행 더 보기 ('+(rows.length-count)+')');
      more.addEventListener('click',()=>{limits.set(key,count+12);fill();});
      items.push(more);
    }
    body.replaceChildren(...items);
  }
  node.open=openedGroups.has(key) || Boolean(searchText.trim()) || Boolean(versionFilter);
  if (node.open) fill();
  node.addEventListener('toggle',()=>{
    if (node.open) {openedGroups.add(key); if (!body.childElementCount) fill();}
    else {openedGroups.delete(key);body.replaceChildren();}
  });
  return node;
}
function draw() {
  if (!latest) return;
  const slam=$('showSlam').checked,truth=$('showTruth').checked;
  for (const card of latest.runs) {
    const node=$('grid').querySelector('.card[data-id="'+card.id+'"]');
    if (node) drawRun(node.querySelector('canvas'),card,scenes.get(card.id),{slam,truth});
  }
  if (openId && openDetail) drawDetail();
}
function refreshGroups() {
  if (!catalog) return;
  const rows=catalog.runs;
  const versions=[...new Set(rows.map(r=>r.version))].sort((a,b)=>a.localeCompare(b,'ko',{numeric:true}));
  const selector=$('versionFilter');
  const options=JSON.stringify(versions);
  if (selector.dataset.options!==options) {
    selector.replaceChildren(el('option',{value:''},'전체 버전'),...versions.map(v=>el('option',{value:v},v)));
    if (versionFilter && !versions.includes(versionFilter)) { versionFilter=''; rememberFilters(); }
    selector.value=versionFilter; selector.dataset.options=options;
  }
  const visible=rows.filter(filtersMatch).sort((a,b)=>b.started-a.started || a.label.localeCompare(b.label));
  const live=visible.filter(r=>r.live);
  latest={runs:live.map(r=>catalog.live[r.id]).filter(Boolean)};
  const liveNodes=latest.runs.map(card=>{
    const node=renderCard(card),row=descriptions.get(card.id);
    node.querySelector('h2').textContent=row.version;
    node.querySelector('.card-head p').textContent=[runName(row),fixtureName(row),recordDate(row)].filter(Boolean).join(' · ');
    node.setAttribute('aria-label',row.version+' '+runName(row)+' 지도와 영상');
    loadScene(card.id); if ($('showSlam').checked) loadMap(card.id,card.snapshot);
    return node;
  });
  const grid=$('grid');
  if (grid.children.length!==liveNodes.length || liveNodes.some((n,i)=>grid.children[i]!==n)) grid.replaceChildren(...liveNodes);
  $('liveSection').hidden=!liveNodes.length;
  const historical=visible.filter(r=>!r.live);
  const key=JSON.stringify([archiveView,versionFilter,searchText,historical.map(r=>[r.id,r.version,r.phase,r.strict,r.approached,r.total,r.sim_time,r.started])]);
  if (historyKey!==key) {
    const groups=new Map();
    for (const row of historical) {
      if (!groups.has(row.version)) groups.set(row.version,[]);
      groups.get(row.version).push(row);
    }
    $('versionGroups').replaceChildren(...[...groups].map(([v,items])=>groupNode(v,items)));
    $('historyCount').textContent=groups.size?groups.size+'개 버전 · '+historical.length+'회':'';
    $('empty').hidden=historical.length>0;
    $('empty').textContent=versionFilter||searchText?'조건에 맞는 주행 기록이 없습니다.':
      (archiveView?'보관한 주행이 없습니다.':'아직 완료된 시험 주행이 없습니다.');
    historyKey=key;
  }
  $('historyTitle').textContent=archiveView?'보관한 버전별 기록':'버전별 주행 기록';
  draw();
}
async function poll() {
  const view=archiveView;
  if (loadingView!==null) {refreshAgain=true;return;}
  loadingView=view;
  try {
    const data=await json('/api/experiments?view='+(view?'archive':'current'));
    if (view!==archiveView) {refreshAgain=true;return;}
    catalog=data;
    $('versionGroups').inert=false;
    $('grid').inert=false;
    $('currentCount').textContent=data.current;
    $('archiveCount').textContent=data.archived;
    $('currentTab').setAttribute('aria-pressed',String(!view));
    $('archiveTab').setAttribute('aria-pressed',String(view));
    $('updated').textContent='갱신 '+new Date().toLocaleTimeString('ko-KR');
    if ($('libraryStatus').dataset.networkError) { $('libraryStatus').textContent=''; delete $('libraryStatus').dataset.networkError; }
    for (const row of data.runs) descriptions.set(row.id,row);
    refreshGroups();
    const id=openId;
    if (id) {
      const detail=await json('/api/run?id='+encodeURIComponent(id));
      if (id===openId) {
        openDetail=detail; loadScene(id); if ($('showSlam').checked) loadMap(id,detail.snapshot); drawDetail();
      }
    }
  } catch(e) {
    $('libraryStatus').dataset.networkError='1';
    $('libraryStatus').textContent='보드 연결을 확인하는 중입니다.';
  } finally {
    loadingView=null;
    if (refreshAgain) { refreshAgain=false; setTimeout(poll,0); }
  }
}
function switchView(next) {
  if (archiveView===next) return;
  archiveView=next; versionFilter=''; searchText=''; $('runSearch').value='';
  $('versionGroups').inert=true; $('grid').inert=true;
  historyKey=''; rememberFilters(); poll();
}
$('currentTab').addEventListener('click',()=>switchView(false));
$('archiveTab').addEventListener('click',()=>switchView(true));
$('versionFilter').addEventListener('change',e=>{versionFilter=e.target.value;rememberFilters();refreshGroups();});
$('runSearch').value=searchText;
$('runSearch').addEventListener('input',e=>{searchText=e.target.value;rememberFilters();refreshGroups();});
