"""Adapt the pinned upstream board's two-marker labels and in-card video playback."""
from pathlib import Path
import sys


dashboard = Path(sys.argv[1]) if len(sys.argv)>1 else Path.home()/'corridor-board-73bbb05/dashboard'
live = dashboard/'web/app.js'
live.write_text(live.read_text().replace(' / 3',' / 2'))
script = dashboard/'sim_web/app.js'
app = script.read_text()
def add_resume(app):
    if '<option value="r32">' not in app:
        anchor = '원본 R32 + 최신 Mixed 동시 주행</option>'
        if anchor not in app:
            raise ValueError('Missing track policy selector')
        app = app.replace(anchor,'R32 Resume + 최신 Mixed 동시 주행</option><option value="r32">R32 Resume 단독 · 5e95333</option>')
    if 'd.rgb_resumptions' not in app:
        anchor = "  $('detailStats').replaceChildren"
        if anchor not in app:
            raise ValueError('Missing run statistics')
        app = app.replace(anchor, """  if (d.policy_revision === '5e9533380aa9233d22656862b2c1b42ce2cdcbf3') {
    stats.push(['주행 정책', 'R32 Resume · 5e95333'], ['RGB 마커 접근 재개', d.rgb_resumptions ?? '수신 대기']);
  }
""" + anchor)
    return app


if 'function experimentCamera' in app:
    script.write_text(add_resume(app))
    sys.exit(0)
app = app.replace('보낸 목표','기록된 탐색 목표').replace(", 지도 ${card.final.map_passed ? '통과' : '미통과'}","")
app = app.replace("아직 기록이 없습니다(30초마다 기록).", "이 주행은 지도 진행률을 기록하지 않았습니다.")
app = app.replace("markers.length : '?'", "markers.length : (card.label.includes('-track-') ? 3 : 2)")
app = app.replace('function drawRun(canvas', 'const trackFloors = new Map();\nfunction drawRun(canvas')
app = app.replace('  if (options.slam) drawSlam', """
  if (options.truth && card.label.includes('-track-')) {
    if (!trackFloors.has(card.id)) {
      const image = new Image();
      image.onload = draw;
      image.src = '/track-floor.jpg?id=' + card.id;
      trackFloors.set(card.id, image);
    }
    const image = trackFloors.get(card.id);
    if (image.complete && image.naturalWidth) {
      const [xmin, xmax, ymin, ymax] = scene.bounds;
      ctx.save(); ctx.clip();
      ctx.drawImage(image, v.X(xmin), v.Y(ymax), (xmax-xmin)*v.s, (ymax-ymin)*v.s);
      ctx.restore();
    }
  }
  if (options.slam) drawSlam""")
app = app.replace("ctx.fillStyle = css('--box');", "ctx.fillStyle = b.name.startsWith('table_') ? '#c5a981' : css('--box');")
anchor = 'function drawDetail() {'
if anchor not in app:
    raise RuntimeError('Unexpected upstream dashboard version')

camera = """
function experimentCamera(d) {
  const map = document.getElementById('bigMap');
  let section = document.getElementById('trial-camera');
  if (!section) {
    section = document.createElement('section');
    section.id = 'trial-camera';
    section.innerHTML = '<h3>로봇 전방 카메라</h3><img alt="로봇 전방 카메라" style="width:100%"><video controls playsinline preload="metadata" style="width:100%" hidden></video>';
    map.parentNode.appendChild(section);
  }
  const image = section.querySelector('img'), video = section.querySelector('video');
  if (section.dataset.run !== d.id) {
    video.pause(); video.removeAttribute('src'); video.load();
    video.hidden = true; image.hidden = false;
    section.dataset.run = d.id; section.dataset.ready = '';
    image.src = '/camera.jpg?id=' + d.id;
  }
  if (d.phase === 'running') image.src = '/camera.jpg?id=' + d.id + '&t=' + Math.floor(Date.now()/1000);
  if (d.phase === 'finished' && !section.dataset.ready) {
    json('/api/media?id=' + d.id).then(media => {
      if (openId !== d.id || !media.ready || section.dataset.ready) return;
      section.dataset.ready = 'yes';
      video.src = '/video.mp4?id=' + d.id;
      video.hidden = false; image.hidden = true;
    });
  }
}
$('detail').addEventListener('close', () => document.querySelector('#trial-camera video')?.pause());
"""
app=app.replace(anchor,camera+'\n'+anchor).replace("  $('detailTitle').textContent = d.label;", "  experimentCamera(d);\n  $('detailTitle').textContent = d.label;")
app += """
const trackActions = document.createElement('form');
trackActions.innerHTML = '<label>트랙 정책 <select><option value="pair">원본 R32 + 최신 Mixed 동시 주행</option><option value="mixed">Mixed</option><option value="flow">기존 Flow v3</option></select></label> <button type="submit">새 랜덤 트랙 주행</button> <span role="status"></span>';
document.querySelector('header').appendChild(trackActions);
trackActions.addEventListener('submit', async event => {
  event.preventDefault();
  const button = trackActions.querySelector('button'), status = trackActions.querySelector('span');
  button.disabled = true;
  try {
    const response = await fetch('/api/start-track', {method:'POST', headers:{'Content-Type':'application/json'},
      body:JSON.stringify({policy:trackActions.querySelector('select').value})});
    const result = await response.json(); status.textContent = result.message || result.error;
  } catch { status.textContent = '보드 연결을 확인해 주세요.'; }
  finally { button.disabled = false; }
});
"""
script.write_text(add_resume(app))
