"""Build a separate import overlay from a pinned adaptive package. Never edit the base."""
import argparse
import hashlib
import os
from pathlib import Path
import shutil
import tempfile

HASHES = {'mission.py':'3647a2dd8a8978cf8d5a645feed18b5b00eb815365a090e98a359b23830047d3',
          'hybrid.py':'e13ce1f90df8ae385c03a69677d6c997bc7e0ec16496771bc4c0c9b519eb56e8',
          'detector.py':'83abf90700dfe53ed7a0067db4e54088464e6e2b80359dfc2e9e569e60f7925d',
          'center_wall.py':'f37cc4258afe9e23e84e70f2ef9420834ce065d2b31318192c8dcdb613f650ce',
          'phased_hybrid.py':'8babff4a3669e3d092bf39d602c2822a3cda98f560b302875f5a4e56ee11ff18'}

def swap(source,old,new):
    if source.count(old)!=1:
        raise ValueError('Installation anchor changed: '+old)
    return source.replace(old,new)

def stage(base,dest,guarded=False):
    if dest.exists():
        raise FileExistsError(dest)
    for name,expected in HASHES.items():
        if hashlib.sha256((base/name).read_bytes()).hexdigest()!=expected:
            raise ValueError('Adaptive source revision changed: '+name)
    with tempfile.TemporaryDirectory(dir=dest.parent) as tmp:
        out=Path(tmp)/'overlay'/'osp_mission'
        shutil.copytree(base,out,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
        for p in ('marker_nbv.py','nbv_rgb.py','nbv_camera.py','nbv_faces.py','guarded_nbv.py'):
            shutil.copy2(Path(__file__).with_name(p),out/p)
        mission=(out/'mission.py').read_text()
        mission=swap(mission,'from osp_mission.phased_hybrid import PhasedHybrid as Hybrid',('from osp_mission.guarded_nbv import GuardedNBV as Hybrid' if guarded else
                      'from osp_mission.marker_nbv import MarkerNBV as Hybrid'))
        after="                    self.camera_diagnostics['two_view_frames']"
        mission=swap(mission,after,"                    from osp_mission.nbv_camera import record_exposure\n"
                     "                    record_exposure(self,m,(camera.transform.translation.x,camera.transform.translation.y,yaw),\n"
                     "                                    endpoints,(mt.translation.x,mt.translation.y,map_yaw(mt.rotation)))\n"+after)
        mission=swap(mission,'and self.marker_focus.request_interrupt', 'and (self.hybrid is None or self.hybrid.interrupt_allowed(self))\n                    and self.marker_focus.request_interrupt')
        # Allow only this policy's anonymous nav/face tasks to be reconsidered. Native marker,
        # hazard, recovery, bearing and spin cooldown protections remain owned by mission.tick.
        old="            if (hybrid is not None and self.kind=='nav' and not protected and not self.cancel_reason"
        mission=swap(mission,old,"            nbv_task = hybrid is not None and (any(k in hybrid.current for k in ('nbv_rgb','nbv_face')) or hybrid.current.get('face_inspection',False))\n"
                     +old.replace('and not protected','and (not protected or nbv_task)'))
        old="                    and not getattr(self.near_pass,'protected_navigation',False)\n                    and hybrid.should_interrupt(self)):"
        mission=swap(mission,old,old.replace("not getattr(self.near_pass,'protected_navigation',False)","(not getattr(self.near_pass,'protected_navigation',False) or nbv_task)"))
        (out/'mission.py').write_text(mission)
        detector=(out/'detector.py').read_text()
        begin=detector.index('            if ids is None:\n                try:\n                    from osp_mission.hybrid import publish_print')
        end=detector.index('            # CameraInfo distortion',begin)
        detector=detector[:begin]+'''            try:
                from osp_mission.nbv_camera import publish
                publish(self,msg,image,corners if ids is not None else None)
            except Exception as exc:
                self.get_logger().debug(f'Anonymous print projection unavailable: {exc}')
            if ids is None:
                return
'''+detector[end:]
        (out/'detector.py').write_text(detector)
        os.rename(out.parent,dest)

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('package',type=Path)
    p.add_argument('overlay',type=Path)
    p.add_argument('--guarded',action='store_true')
    a=p.parse_args()
    stage(a.package,a.overlay,a.guarded)
