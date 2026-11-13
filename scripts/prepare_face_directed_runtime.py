"""Stage observed-face orientation priority without editing active runtimes."""
import argparse
from pathlib import Path
import shutil

from prepare_approach_witness_runtime import replace_once

ROOT=Path(__file__).resolve().parents[1]


def prepare(source,destination):
    if destination.exists():raise FileExistsError('Use a new immutable source stage')
    package=destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission',package,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    shutil.copy2(ROOT/'scripts/candidate_face_directed_look.py',package/'osp_mission/face_directed_look.py')
    shutil.copy2(ROOT/'scripts/test_face_directed_runtime.py',package/'test/test_face_directed_runtime.py')
    path=package/'osp_mission/mission.py';code=path.read_text()
    code=replace_once(code,'from osp_mission.marker_directed_look import MarkerDirectedLook',
        'from osp_mission.marker_directed_look import MarkerDirectedLook\nfrom osp_mission.face_directed_look import FaceDirectedLook')
    code=replace_once(code,'        self.marker_look=MarkerDirectedLook(self)',
        '        self.marker_look=MarkerDirectedLook(self)\n        self.face_look=FaceDirectedLook(self)')
    code=replace_once(code,'        if directed is not None:return directed\n',
        "        if directed is not None:return directed\n        face=getattr(self,'face_look',None)\n        directed=face.consume() if face is not None else None\n        if directed is not None:return directed\n")
    lines=code.splitlines();result=[]
    for line in lines:
        result.append(line)
        if 'self.marker_look.clear(' in line:
            mirrored=line.replace('marker_look','face_look')
            if line.lstrip().startswith('self.marker_look.clear('):
                indent=line[:len(line)-len(line.lstrip())]
                mirrored=indent+"if getattr(self,'face_look',None) is not None:"+mirrored.lstrip()
            result.append(mirrored)
    code='\n'.join(result)+'\n'
    code=replace_once(code,'        self.marker_look.select(selected,memory_transform)',
        '        self.marker_look.select(selected,memory_transform)\n        self.face_look.select(selected,memory_transform)')
    code=replace_once(code,"report={'marker_directed_look':", "report={'face_directed_look':self.face_look.report(),'marker_directed_look':")
    path.write_text(code)
    return destination


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('source','destination'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();print(prepare(args.source.resolve(),args.destination.resolve()))
