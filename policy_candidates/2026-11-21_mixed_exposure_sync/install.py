"""Create an independent v1.5.1 sensor-synchronized stage from frozen v1.5.0."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


def replace(text,old,new):
    if text.count(old)!=1:raise ValueError('Unexpected frozen anchor: '+old[:100])
    return text.replace(old,new,1)


def install(source,destination):
    if destination.exists():raise FileExistsError(destination)
    here=Path(__file__).resolve().parent
    with tempfile.TemporaryDirectory(dir=destination.parent) as temporary:
        stage=Path(temporary)/'workspace';package=stage/'src/osp_mission'
        shutil.copytree(source/'src/osp_mission',package,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
        modules=package/'osp_mission';detector=modules/'detector.py';text=detector.read_text()
        text=replace(text,'        self.pipeline_timing=PipelineTiming()',
            '        self.pipeline_timing=PipelineTiming()\n        from osp_mission.sensor_history import SensorHistory\n        self.sensor_history=SensorHistory()')
        text=replace(text,'    def on_scan(self,m): self.scan=m',
            '    def on_scan(self,m):\n        self.scan=m;self.sensor_history.scan(m)')
        text=replace(text,'    def on_depth(self,m): self.depth=m',
            '    def on_depth(self,m):\n        self.depth=m;self.sensor_history.depth(m)')
        text=replace(text,'        tr=self.tf.lookup_transform(scan.header.frame_id,frame,stamp,timeout=Duration(seconds=0.))',
            '''        tr=self.tf.lookup_transform_full(scan.header.frame_id,Time.from_msg(scan.header.stamp),
            frame,stamp,self.p('odom_frame'),timeout=Duration(seconds=0.))''')
        text=replace(text,'        latest=self.info;self.info=row.calibration;started=time.perf_counter()',
            '''        latest_info,latest_scan,latest_depth=self.info,self.scan,self.depth
        self.info=row.calibration
        self.scan,self.depth=self.sensor_history.select(row.message)
        started=time.perf_counter()''')
        text=replace(text,'            self.info=latest\n',
            '            self.info,self.scan,self.depth=latest_info,latest_scan,latest_depth\n')
        text=replace(text,'            pipeline=self.pipeline_timing.report(),',
            '            pipeline=self.pipeline_timing.report(),sensor_history=self.sensor_history.report(),')
        detector.write_text(text);shutil.copy2(here/'sensor_history.py',modules/'sensor_history.py')
        work=modules/'camera_work.py';text=work.read_text().replace("'processed'","'dispatched'").replace('processed=0','dispatched=0').replace('processed_fps_ros=','dispatched_fps_ros=');work.write_text(text)
        hashes={}
        original=source/'src/osp_mission'
        for path in original.rglob('*'):
            if not path.is_file() or '__pycache__' in path.parts or '.pytest_cache' in path.parts:continue
            relative=path.relative_to(original)
            if str(relative) in ('osp_mission/detector.py','osp_mission/camera_work.py'):continue
            assert path.read_bytes()==(package/relative).read_bytes(),relative
            hashes[str(relative)]=hashlib.sha256(path.read_bytes()).hexdigest()
        (stage/'exposure_sync_manifest.json').write_text(json.dumps(dict(version='1.5.1',base='1.5.0',
            unchanged_sha256=hashes,changed=['osp_mission/detector.py','osp_mission/camera_work.py'],
            added=['osp_mission/sensor_history.py']),indent=2)+'\n')
        subprocess.run(['python3','-m','compileall','-q',str(modules)],check=True)
        stage.rename(destination)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frozen_camera_aware',type=Path);parser.add_argument('new_workspace',type=Path)
    args=parser.parse_args();install(args.frozen_camera_aware.resolve(),args.new_workspace.resolve())
