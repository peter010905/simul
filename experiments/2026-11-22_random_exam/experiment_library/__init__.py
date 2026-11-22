"""A versioned list for the shared 8780 board, preserving its map, media and trial APIs."""
import json
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from .catalog import Library


HERE=Path(__file__).resolve().parent


def between(source,before,after,replacement):
    if source.count(before)!=1 or source.count(after)!=1:
        raise ValueError('Unexpected upstream board layout around '+before[:55])
    a=source.index(before)
    b=source.index(after,a)
    return source[:a]+replacement+source[b:]


def replace(source,old,new):
    if source.count(old)!=1:
        raise ValueError('Unexpected upstream board layout around '+old[:55])
    return source.replace(old,new,1)


def assets(web):
    html=between((web/'index.html').read_text(),'  <header','  <dialog',(HERE/'header.html').read_text())
    js=between((web/'app.js').read_text(),'function draw() {','// ---------------------------------------------------------------- detail view',
               (HERE/'library.js').read_text()+'\n')
    js=between(js,'function openRun(id) {',"$('close').addEventListener",'''async function openRun(id) {
  openId=id; openDetail=null;
  if (!$('detail').open) $('detail').showModal();
  const row=descriptions.get(id);
  $('detailTitle').textContent=row ? row.version : '주행 기록';
  $('detailWhere').textContent='기록을 불러오는 중입니다.';
  try {
    const d=await json('/api/run?id='+encodeURIComponent(id));
    if (openId!==id) return;
    openDetail=d;
    await Promise.all([loadScene(id),$('showSlam').checked?loadMap(id,d.snapshot):Promise.resolve()]);
    if (openId===id) drawDetail();
  } catch { if (openId===id) $('detailWhere').textContent='주행 기록을 불러오지 못했습니다.'; }
}
''')
    js=replace(js,"  $('detailTitle').textContent = d.label;", "  $('detailTitle').textContent = descriptions.get(d.id)?.version || d.label;")
    js=replace(js,"  $('detailWhere').textContent = `${d.where} · ${PHASE[d.phase] || d.phase}`;",
               "  $('detailWhere').textContent = d.label+' · '+(displayPhases[descriptions.get(d.id)?.phase || d.phase] || d.phase);")
    js=replace(js,"['시뮬 시간', `${clock(d.sim_time)} / ${clock(d.duration)}`]",
               "['시뮬 시간', `${clock(d.phase==='finished' ? descriptions.get(d.id)?.sim_time ?? d.sim_time : d.sim_time)} / ${clock(d.duration)}`]")
    js=replace(js,"$('showAll').addEventListener('change', poll);","")
    js=replace(js,'setInterval(poll, 1000);','setInterval(poll, 2100);')
    css=(web/'style.css').read_text()+'\n'+(HERE/'library.css').read_text()
    return {k:(content.encode(),mime) for k,content,mime in (
        ('/index.html',html,'text/html; charset=utf-8'),('/app.js',js,'text/javascript; charset=utf-8'),
        ('/style.css',css,'text/css; charset=utf-8'))}


def grouped_handler(parent,board,args,web):
    static=assets(web)
    library=Library(board,args)

    class Handler(parent):
        def do_GET(self):
            url=urlsplit(self.path)
            if url.path=='/api/experiments':
                view=parse_qs(url.query).get('view',['current'])[0]
                if view not in ('current','archive'):
                    return self.send(400,{'error':'알 수 없는 보기입니다.'})
                return self.send(200,library.list(view=='archive'))
            resource=static.get('/index.html' if url.path=='/' else url.path)
            if resource:
                return self.send(200,*resource)
            return super().do_GET()

        def do_POST(self):
            if self.path!='/api/experiment-visibility':
                return super().do_POST()
            host=self.headers.get('Host','')
            if (host not in {f'{args.host}:8780','localhost:8780','127.0.0.1:8780'} or
                    self.headers.get('Origin')!='http://'+host):
                return self.send(403,{'error':'이 실험 보드에서 변경해 주세요.'})
            length=self.headers.get('Content-Length','')
            if self.headers.get_content_type()!='application/json' or not length.isdecimal() or int(length)>256:
                return self.send(400,{'error':'잘못된 보관 요청입니다.'})
            try:
                data=json.loads(self.rfile.read(int(length)))
                if not isinstance(data,dict) or not isinstance(data.get('id'),str) or type(data.get('archived')) is not bool:
                    raise ValueError('주행과 보관 여부를 지정해 주세요.')
                library.set_archived(data['id'],data['archived'])
            except KeyError:
                return self.send(404,{'error':'해당 주행을 찾을 수 없습니다.'})
            except ValueError as error:
                return self.send(409 if str(error).startswith('진행 중') else 400,{'error':str(error)})
            except OSError:
                return self.send(503,{'error':'보관 설정을 저장할 수 없습니다.'})
            return self.send(200,{'archived':data['archived']})

    return Handler
