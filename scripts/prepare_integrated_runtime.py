"""Compose isolated motion,viewing,and final-map candidates in a new stage.

Building this source does not qualify it as a successful policy. Original
stages and current native experiments remain immutable and untouched.
"""
import argparse
from pathlib import Path
import shutil
import subprocess

from prepare_approach_witness_runtime import replace_once
from prepare_velocity_guard_runtime import prepare as prepare_guard
from prepare_encoder_delta_runtime import prepare as prepare_motion
from prepare_near_pass_runtime import prepare as prepare_inspection

ROOT=Path(__file__).resolve().parents[1]


def finalize_graph(source,destination):
    if destination.exists():raise FileExistsError('Use a new immutable integrated candidate')
    package=destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission',package,
        ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    shutil.copy2(ROOT/'scripts/candidate_graph_landmarks.py',package/'osp_mission/graph_landmarks.py')
    shutil.copy2(ROOT/'scripts/test_graph_landmarks_runtime.py',package/'test/test_graph_landmarks_runtime.py')
    tools=package/'tools';tools.mkdir(exist_ok=True)
    shutil.copy2(ROOT/'training_robust/export_slam_graph_poses.cpp',tools/'export_slam_graph_poses.cpp')
    subprocess.run(['g++','-std=c++17','-O1','-I/opt/ros/jazzy/include','-I/usr/include/eigen3',
        str(tools/'export_slam_graph_poses.cpp'),'-L/opt/ros/jazzy/lib','-Wl,-rpath,/opt/ros/jazzy/lib',
        '-lkartoSlamToolbox','-lboost_serialization','-lboost_thread','-lpthread',
        '-o',str(tools/'export_slam_graph_poses')],check=True,capture_output=True,text=True)
    path=package/'setup.py';path.write_text(replace_once(path.read_text(),
        "['launch', 'config', 'worlds', 'rviz']","['launch', 'config', 'worlds', 'rviz', 'tools']"))
    path=package/'osp_mission/mission.py';code=path.read_text()
    code=replace_once(code,'from osp_mission.marker_approach import MarkerApproach',
        'from osp_mission.marker_approach import MarkerApproach\n'
        'from osp_mission.graph_landmarks import finalize_output\n'
        'from ament_index_python.packages import get_package_share_directory')
    code=replace_once(code,'        self.approach=MarkerApproach()',
        '        self.approach=MarkerApproach()\n        self.graph_observations=[]')
    code=replace_once(code,"    def on_approach_image(self,m):\n        if self.state!='running':return\n        try:\n            evidence=json.loads(m.data)\n            stamp=float(evidence['stamp'])",
        "    def on_approach_image(self,m):\n        if self.state!='running':return\n        try:\n            evidence=json.loads(m.data)\n"
        "            self.graph_observations.append({key:evidence.get(key) for key in\n"
        "                ('id','stamp','odom_point','min_edge_pixels','provisional')})\n"
        "            del self.graph_observations[:-30000]\n            stamp=float(evidence['stamp'])")
    code=replace_once(code,"        self.save_log.close()\n        files=['map.pgm','map.yaml','map.posegraph','map.data','markers.csv']",
        "        self.save_log.close()\n"
        "        self.graph_landmark_update=dict(applied=False,ground_truth_inputs=False,\n"
        "            corrected_ids=[],reason='SLAM_serialization_not_completed')\n"
        "        try:\n"
        "            if self.serialize_future and self.serialize_future.done() and self.serialize_future.result().result==0:\n"
        "                exporter=Path(get_package_share_directory('osp_mission'))/'tools/export_slam_graph_poses'\n"
        "                self.targets,self.graph_landmark_update=finalize_output(\n"
        "                    self.graph_observations,self.targets,self.output,exporter)\n"
        "        except Exception as exc:\n"
        "            self.graph_landmark_update.update(error=repr(exc),original_coordinates_preserved=True)\n"
        "        files=['map.pgm','map.yaml','map.posegraph','map.data','markers.csv']")
    code=replace_once(code,"report={'near_pass_inspection':", "report={'graph_landmark_update':self.graph_landmark_update,'near_pass_inspection':")
    path.write_text(code)
    return destination


def prepare(source,destination):
    parent=destination.parent
    if parent.exists():raise FileExistsError('Use a new stage parent;preserve partial preparation evidence')
    parent.mkdir(parents=True)
    guard=prepare_guard(source,parent/'guard_source')
    motion=prepare_motion(guard,parent/'motion_source')
    inspection=prepare_inspection(motion,parent/'inspection_source')
    return finalize_graph(inspection,destination)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('source','destination'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();print(prepare(args.source.resolve(),args.destination.resolve()))
