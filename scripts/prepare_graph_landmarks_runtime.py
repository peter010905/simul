"""Stage final SLAM graph landmark updates without changing native driving."""
import argparse
from pathlib import Path
import shutil
import subprocess

from prepare_approach_witness_runtime import replace_once

ROOT=Path(__file__).resolve().parents[1]


def prepare(source,destination):
    if destination.exists():raise FileExistsError('Use a new immutable runtime stage')
    package=destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission',package,
        ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    shutil.copy2(ROOT/'scripts/candidate_graph_landmarks.py',package/'osp_mission/graph_landmarks.py')
    shutil.copy2(ROOT/'scripts/test_graph_landmarks_runtime.py',package/'test/test_graph_landmarks_runtime.py')
    tools=package/'tools';tools.mkdir(exist_ok=True)
    shutil.copy2(ROOT/'training_robust/export_slam_graph_poses.cpp',tools/'export_slam_graph_poses.cpp')
    command=['g++','-std=c++17','-O1','-I/opt/ros/jazzy/include','-I/usr/include/eigen3',
        str(tools/'export_slam_graph_poses.cpp'),'-L/opt/ros/jazzy/lib','-Wl,-rpath,/opt/ros/jazzy/lib',
        '-lkartoSlamToolbox','-lboost_serialization','-lboost_thread','-lpthread',
        '-o',str(tools/'export_slam_graph_poses')]
    subprocess.run(command,check=True,capture_output=True,text=True)
    path=package/'setup.py'
    path.write_text(replace_once(path.read_text(),"['launch', 'config', 'worlds', 'rviz']",
        "['launch', 'config', 'worlds', 'rviz', 'tools']"))
    path=package/'osp_mission/mission.py';code=path.read_text()
    code=replace_once(code,'from osp_mission.marker_approach import MarkerApproach',
        'from osp_mission.marker_approach import MarkerApproach\n'
        'from osp_mission.graph_landmarks import finalize_output\n'
        'from ament_index_python.packages import get_package_share_directory')
    code=replace_once(code,'        self.approach=MarkerApproach()',
        '        self.approach=MarkerApproach()\n        self.graph_observations=[]')
    old='''    def on_approach_image(self,m):
        if self.state!='running':return
        try:
            evidence=json.loads(m.data)
            stamp=float(evidence['stamp'])'''
    new='''    def on_approach_image(self,m):
        if self.state!='running':return
        try:
            evidence=json.loads(m.data)
            # Preserve only accepted sensor/odom evidence, never scene truth.
            self.graph_observations.append({key:evidence.get(key) for key in
                ('id','stamp','odom_point','min_edge_pixels','provisional')})
            del self.graph_observations[:-30000]
            stamp=float(evidence['stamp'])'''
    code=replace_once(code,old,new)
    old='''        self.save_log.close()
        files=['map.pgm','map.yaml','map.posegraph','map.data','markers.csv']'''
    new='''        self.save_log.close()
        self.graph_landmark_update=dict(applied=False,ground_truth_inputs=False,
            corrected_ids=[],reason='SLAM_serialization_not_completed')
        try:
            if self.serialize_future and self.serialize_future.done() and self.serialize_future.result().result==0:
                exporter=Path(get_package_share_directory('osp_mission'))/'tools/export_slam_graph_poses'
                self.targets,self.graph_landmark_update=finalize_output(
                    self.graph_observations,self.targets,self.output,exporter)
        except Exception as exc:
            self.graph_landmark_update.update(error=repr(exc),original_coordinates_preserved=True)
        files=['map.pgm','map.yaml','map.posegraph','map.data','markers.csv']'''
    code=replace_once(code,old,new)
    code=replace_once(code,"report={'approach':self.approach.report(),'reason':self.reason,",
        "report={'graph_landmark_update':self.graph_landmark_update,'approach':self.approach.report(),'reason':self.reason,")
    path.write_text(code)
    return destination


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('source','destination'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();print(prepare(args.source.resolve(),args.destination.resolve()))
