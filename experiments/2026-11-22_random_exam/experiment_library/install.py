"""Install an immutable view module beside an existing simulation display launcher."""
import argparse
import hashlib
import os
from pathlib import Path
import shutil
import tempfile


HERE=Path(__file__).resolve().parent
FILES=('__init__.py','catalog.py','header.html','library.js','library.css')
BEGIN='    # BEGIN SIMULATION EXPERIMENT LIBRARY'
END='    # END SIMULATION EXPERIMENT LIBRARY'
LEGACY='    ThreadingHTTPServer((args.host,8780),Handler).serve_forever()'
MEDIA="                folder=(args.track_source if '-track-' in run.label else args.source)/run.label"
RESOLVED="                folder=recording_folder((args.track_source,args.cards.parent/'recordings',args.source),run.label,args.source/run.label)"


def install(launcher):
    original=launcher.read_text()
    current=original
    if BEGIN in current:
        if current.count(BEGIN)!=1 or current.count(END)!=1:
            raise ValueError('Incomplete display integration')
        a=current.index(BEGIN)
        b=current.index(END,a)+len(END)
        current=current[:a]+LEGACY+current[b:]
    if current.count(LEGACY)!=1:
        raise ValueError('Unexpected upstream display listener')
    if MEDIA in current:
        current=current.replace(MEDIA,RESOLVED,1)
    if RESOLVED not in current:
        raise ValueError('Unexpected camera recording resolver')
    content={f:(HERE/f).read_bytes() for f in FILES}
    key=hashlib.sha256(b''.join(f.encode()+b'\0'+b for f,b in content.items())).hexdigest()[:16]
    root=launcher.parent/'.board-versions'/key
    package=root/'experiment_library'
    if not package.exists():
        root.parent.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=root.parent) as temp:
            staged=Path(temp)/'staged'
            (staged/'experiment_library').mkdir(parents=True)
            for fname,data in content.items():
                (staged/'experiment_library'/fname).write_bytes(data)
            staged.rename(root)
    if any((package/f).read_bytes()!=data for f,data in content.items()):
        raise RuntimeError('Changed display version')
    block='\n'.join((BEGIN,
        f"    sys.path.insert(0,str(Path(__file__).resolve().parent/'.board-versions'/'{key}'))",
        '    from experiment_library import grouped_handler',
        '    from experiment_library.catalog import recording_folder',
        '    from sim_board import WEB',
        '    ThreadingHTTPServer((args.host,8780),grouped_handler(Handler,board,args,WEB)).serve_forever()',END))
    updated=current.replace(LEGACY,block)
    compile(updated,str(launcher),'exec')
    if updated!=original:
        backup=launcher.with_name(launcher.name+'.before-version-library')
        if not backup.exists():
            shutil.copy2(launcher,backup)
        with tempfile.NamedTemporaryFile(dir=launcher.parent,delete=False) as stream:
            stream.write(updated.encode())
            temp=Path(stream.name)
        temp.chmod(launcher.stat().st_mode&0o777)
        if launcher.read_text()!=original:
            temp.unlink()
            raise RuntimeError('Display changed during installation')
        os.replace(temp,launcher)
    return root


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('launcher',type=Path)
    print(install(cli.parse_args().launcher.resolve()))
