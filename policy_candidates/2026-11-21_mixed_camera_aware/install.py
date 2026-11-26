"""Install camera-aware v1.5 into a fresh copy of a frozen v1.2 workspace."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


def replace(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Unexpected frozen source anchor: '+old[:100])
    return text.replace(old, new, 1)


def patch_detector(text):
    text = replace(text, 'import json\n', 'import json\nimport time\n')
    text = replace(text, '        self.visual_range=VisualRange()', '''        from osp_mission.camera_multiview import CameraVisualRange
        from osp_mission.camera_work import ExposureQueue, PipelineTiming
        self.visual_range=CameraVisualRange()
        self.exposure_queue=ExposureQueue()
        self.pipeline_timing=PipelineTiming()''')
    text = replace(text, "self.create_subscription(Image,self.p('rgb_topic'),self.on_rgb,latest_image)",
                   "self.create_subscription(Image,self.p('rgb_topic'),self.enqueue_rgb,latest_image)\n        self.create_timer(.02,self.drain_rgb)")
    text = text.replace('timeout=Duration(seconds=.05)', 'timeout=Duration(seconds=0.)')
    text = replace(text, '''        # The hardware square preview has a measured horizontal focal correction.
        ray = ray.copy()
        ray[0] *= self.info.k[0] / preview_geometry(self.info).k[0]
''', '''        # on_rgb already used the calibrated horizontal model once, including
        # LiDAR, triangulation and processed-camera coverage geometry.
''')
    anchor = '    def on_rgb(self,msg):'
    helpers = '''    def enqueue_rgb(self,msg):
        if self.info is None:return
        if (self.info.width,self.info.height)!=(msg.width,msg.height):
            self.diagnostics['calibration_mismatch']=self.diagnostics.get('calibration_mismatch',0)+1
            return
        self.exposure_queue.offer(msg,self.info,Time.from_msg(msg.header.stamp).nanoseconds/1e9,
                                  self.get_clock().now().nanoseconds/1e9)

    def drain_rgb(self):
        now=self.get_clock().now().nanoseconds/1e9
        def ready(row):
            frame=row.message.header.frame_id;stamp=Time.from_msg(row.message.header.stamp)
            return all(self.tf.can_transform(self.p(target),frame,stamp)
                       for target in ('map_frame','odom_frame'))
        row=self.exposure_queue.take(now,ready)
        if row is None:return
        latest=self.info;self.info=row.calibration;started=time.perf_counter()
        try:self.on_rgb(row.message)
        finally:
            self.info=latest
            self.pipeline_timing.record(row.stamp,self.get_clock().now().nanoseconds/1e9,time.perf_counter()-started)

'''
    text = replace(text, anchor, helpers+anchor)
    text = replace(text, "        if self.info is None: return\n        try:",
                   "        if self.info is None: return\n        processed=None\n        try:")
    text = replace(text, '            k=self.info.k', '            k=preview_geometry(self.info).k')
    text = replace(text, '                    from osp_mission.hybrid_geometry import preview_geometry\n', '')
    text = replace(text, '            self.processed_camera.publish(processed)\n',
                   '            processed.k=list(k)\n')
    start = text.index('                key=str(int(ident));', text.index('            for c,ident in zip('))
    end = text.index('            if accepted_quads:', start)
    body = text[start:end]
    text = text[:start]+'                try:\n'+''.join('    '+line if line.strip() else line for line in body.splitlines(keepends=True))+'''                except Exception as exc:
                    self.diagnostics['marker_errors']=self.diagnostics.get('marker_errors',0)+1
                    self.diagnostics['last_marker_error']=str(exc)
                    self.get_logger().debug(f'ID {int(ident)} skipped: {exc}')
'''+text[end:]
    text = replace(text, "            self.get_logger().debug(str(exc))\n\n    def publish(self):", '''            self.get_logger().debug(str(exc))
        finally:
            if processed is not None:self.processed_camera.publish(processed)

    def publish(self):''')
    text = replace(text, '            by_id=self.id_diagnostics,', '''            by_id=self.id_diagnostics,
            exposure_queue=dict(self.exposure_queue.counts,pending=len(self.exposure_queue.rows)),
            pipeline=self.pipeline_timing.report(),''')
    return text


def install(source, destination):
    if destination.exists():raise FileExistsError(destination)
    here=Path(__file__).resolve().parent
    with tempfile.TemporaryDirectory(dir=destination.parent) as temporary:
        stage=Path(temporary)/'workspace'
        package=stage/'src/osp_mission'
        shutil.copytree(source/'src/osp_mission',package,
                        ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
        modules=package/'osp_mission'
        detector=modules/'detector.py'
        detector.write_text(patch_detector(detector.read_text()))
        for name in ('camera_work.py','camera_multiview.py'):
            shutil.copy2(here/name,modules/name)
        # Share the exact tested sensor configurator; no navigation overrides.
        bench=here.parents[1]/'experiments/2026-11-20_v12_camera_rate'
        import importlib.util
        spec=importlib.util.spec_from_file_location('sensor_installer',bench/'install.py')
        sensor=importlib.util.module_from_spec(spec);spec.loader.exec_module(sensor)
        with tempfile.TemporaryDirectory(dir=destination.parent) as camera_tmp:
            camera_stage=Path(camera_tmp)/'sensor_stage'
            sensor.install(source,camera_stage)
            shutil.copy2(camera_stage/'src/osp_mission/osp_mission/sensor_model.py',modules/'sensor_model.py')
        changed={'osp_mission/detector.py','osp_mission/sensor_model.py'}
        original=source/'src/osp_mission';hashes={}
        for path in original.rglob('*'):
            if not path.is_file() or '__pycache__' in path.parts or '.pytest_cache' in path.parts:continue
            relative=path.relative_to(original)
            if str(relative) in changed:continue
            assert path.read_bytes()==(package/relative).read_bytes(),relative
            hashes[str(relative)]=hashlib.sha256(path.read_bytes()).hexdigest()
        (stage/'camera_aware_manifest.json').write_text(json.dumps(dict(version='1.5.0',base='v1.2',
            unchanged_sha256=hashes,changed=sorted(changed),added=['camera_work.py','camera_multiview.py']),indent=2)+'\n')
        subprocess.run(['python3','-m','compileall','-q',str(modules)],check=True)
        stage.rename(destination)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frozen_v12',type=Path);parser.add_argument('new_workspace',type=Path)
    args=parser.parse_args();install(args.frozen_v12.resolve(),args.new_workspace.resolve())
