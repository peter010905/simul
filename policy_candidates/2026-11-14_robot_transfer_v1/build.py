"""Build this exact source release in a new workspace; never starts a robot."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess


def build(workspace,run_tests,baseline=False):
    candidate=Path(__file__).resolve().parent
    release=candidate.parents[1]/'policy_releases/2026-11-13_face_intent_v2'
    manifest=json.loads((release/'release.json').read_text())
    for relative,expected in manifest['files'].items():
        if hashlib.sha256((release/relative).read_bytes()).hexdigest()!=expected:
            raise ValueError(f'Release source changed: {relative}')
    candidate_manifest=json.loads((candidate/'candidate.json').read_text())
    for relative,expected in candidate_manifest['overlay_files'].items():
        if hashlib.sha256((candidate/'overlay'/relative).read_bytes()).hexdigest()!=expected:
            raise ValueError(f'Candidate source changed: {relative}')
    if workspace.exists():raise FileExistsError('Choose a new workspace;existing experiments are preserved')
    package=workspace/'src/osp_mission'
    shutil.copytree(release/'runtime/src/osp_mission',package)
    for source in (candidate/'overlay').rglob('*'):
        if source.is_file() and not {'__pycache__','.pytest_cache'}.intersection(source.parts):
            target=package/source.relative_to(candidate/'overlay');target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(source,target)
    if baseline:
        # Identical sensor/transport/launch plumbing; old exploration policy.
        for relative in ['osp_mission/mission.py','osp_mission/edge_policy.py','osp_mission/escape_connectivity.py','osp_mission/exploration_memory.py','test/test_spatial_runtime.py']:
            shutil.copy2(release/'runtime/src/osp_mission'/relative,package/relative)
        (package/'test/test_robot_transfer.py').unlink()
    subprocess.run(['c++','-std=c++17','-O3','-ffp-contract=off','-fno-fast-math','-shared','-fPIC',
        str(package/'osp_mission/raycast.cpp'),'-o',str(package/'osp_mission/_raycast.so')],check=True)
    subprocess.run(['g++','-std=c++17','-O1','-I/opt/ros/jazzy/include','-I/usr/include/eigen3',
        str(package/'tools/export_slam_graph_poses.cpp'),'-L/opt/ros/jazzy/lib','-Wl,-rpath,/opt/ros/jazzy/lib',
        '-lkartoSlamToolbox','-lboost_serialization','-lboost_thread','-lpthread',
        '-o',str(package/'tools/export_slam_graph_poses')],check=True)
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1')
    for key in ('AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','CMAKE_PREFIX_PATH','PYTHONPATH','LD_LIBRARY_PATH','ROS_PACKAGE_PATH'):
        env.pop(key,None)
    command='source /opt/ros/jazzy/setup.bash\ncolcon build --symlink-install --base-paths src --packages-select osp_mission\n'
    command+='source '+shlex.quote(str(workspace/'install/setup.bash'))+'\n'
    command+="python3 -c 'from osp_mission.raycast import native_function; native_function(); from osp_mission.face_directed_look import FaceDirectedLook; print(\"Candidate helpers verified\")'\n"
    if run_tests:command+='python3 -m pytest -q '+shlex.quote(str(package/'test'))+'\n'
    with (workspace/'release_build.log').open('w') as stream:
        subprocess.run(['/bin/bash','-eo','pipefail','-c',command],cwd=workspace,env=env,stdout=stream,stderr=stream,check=True)
    print(workspace/'install/setup.bash')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('--test',action='store_true')
    parser.add_argument('--baseline',action='store_true',help='Old exploration with identical synthetic input/transport plumbing')
    args=parser.parse_args();build(args.workspace.resolve(),args.test,args.baseline)
