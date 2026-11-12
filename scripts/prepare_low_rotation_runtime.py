"""Create an isolated low-rotation candidate; never rewrite a frozen runtime."""
import argparse
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]


def patch_mission(code):
    old="""            self.scan_after=False; goal=Spin.Goal()
            goal.target_yaw=(2*math.pi if self.initial_scan else self.next_scan_angle) if self.surface_mode else (2*math.pi if self.initial_scan else 2*math.pi/3) if self.inspect else scan_angle(self.weights)
            self.initial_scan=False; goal.time_allowance.sec=45"""
    new="""            self.scan_after=False; goal=Spin.Goal()
            goal.target_yaw=self.observation_turn()
            self.initial_scan=False; goal.time_allowance.sec=15
            self.get_logger().info(f'Bounded observation turn: {math.degrees(goal.target_yaw):.0f} deg')"""
    if code.count(old)!=1: raise ValueError('Review changed mission scan block before patching')
    code=code.replace(old,new)
    code=code.replace('from osp_mission.result_io import write_result',
                      'from osp_mission.observation_sweep import select as select_observation_sweep\nfrom osp_mission.result_io import write_result',1)
    anchor='    def load_configuration(self,path):'
    method='''    def observation_turn(self):
        # Older planners keep a bounded inspection; edge planners use camera evidence.
        if not self.edge_mode:
            requested=(2*math.pi/3 if self.initial_scan else self.next_scan_angle) if self.surface_mode or self.inspect else scan_angle(self.weights)
            return math.copysign(min(abs(requested),2*math.pi/3),requested)
        calibration=getattr(self,'inspection_camera',None)
        if self.map is None or calibration is None:return 0.
        try:
            tr=self.tf.lookup_transform(self.p('map_frame'),self.inspection_camera_frame,Time())
            q=tr.transform.rotation
            heading=math.atan2(2*(q.y*q.z-q.w*q.x),2*(q.x*q.z+q.w*q.y))
            m=self.map;data=np.asarray(m.data).reshape(m.info.height,m.info.width)
            origin=(m.info.origin.position.x,m.info.origin.position.y)
            viewed=self.coverage.grid(data,m.info.resolution,origin)
            # Observation coverage still requires exact camera TF and real decoded frames.
            return select_observation_sweep(data,m.info.resolution,origin,
                (tr.transform.translation.x,tr.transform.translation.y,heading),viewed,*calibration)
        except Exception as exc:
            self.get_logger().debug(f'Observation sweep unavailable: {exc}')
            return 0.

'''
    if code.count(anchor)!=1: raise ValueError('Missing configuration anchor')
    code=code.replace(anchor,method+anchor)
    anchor="        self.camera_stamp=seconds;self.pending_camera=m"
    if code.count(anchor)!=1: raise ValueError('Missing camera calibration anchor')
    code=code.replace(anchor,anchor+"\n        self.inspection_camera=(float(m.k[0]),2*math.atan(m.width/(2*m.k[0])))\n        self.inspection_camera_frame=m.header.frame_id")
    # A failed navigation has not reached a useful inspection pose.
    code=code.replace('self.excluded.append(self.destination); self.scan_after=True',
                      'self.excluded.append(self.destination); self.scan_after=result.status==4')
    return code


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--destination',type=Path,required=True)
    args=parser.parse_args()
    if args.destination.exists(): raise SystemExit('Use a new destination; existing runtime is preserved')
    source=ROOT/'training_robust/native90/runtime_fix_v13_observation_staging/src/osp_mission'
    subprocess.run([sys.executable,str(ROOT/'scripts/prepare_robot_runtime.py'),
                    '--source',str(source),'--destination',str(args.destination)],check=True)
    package=args.destination/'src/osp_mission'
    mission=package/'osp_mission/mission.py'
    mission.write_text(patch_mission(mission.read_text()))
    shutil.copy2(ROOT/'src/osp_mission/osp_mission/observation_sweep.py',package/'osp_mission')
    shutil.copy2(ROOT/'src/osp_mission/test/test_observation_sweep.py',package/'test')
    shutil.copy2(ROOT/'src/osp_mission/test/test_recovery.py',package/'test')
    print(f'Unverified low-rotation candidate prepared: {args.destination}')

if __name__=='__main__': main()
