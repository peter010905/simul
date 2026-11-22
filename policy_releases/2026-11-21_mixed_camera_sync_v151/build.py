"""Build the tested text snapshot in a new Jazzy workspace; never start nodes."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


def build(destination):
    here=Path(__file__).resolve().parent
    manifest=json.loads((here/'source_manifest.json').read_text())
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    if destination.exists():raise FileExistsError('Choose a new workspace; existing runs are preserved')
    for relative,expected in manifest['source_sha256'].items():
        assert digest(here/'src/osp_mission'/relative)==expected,relative
    for relative,expected in manifest['external_config_sha256'].items():
        assert digest(here/relative)==expected,relative
    shutil.copytree(here/'src',destination/'src',ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    configs=destination/'frozen-config';configs.mkdir()
    for name in manifest['external_config_sha256']:shutil.copy2(here/name,configs/name)
    package=destination/'src/osp_mission'
    subprocess.run(['c++','-std=c++17','-O3','-ffp-contract=off','-fno-fast-math','-shared','-fPIC',
        str(package/'osp_mission/raycast.cpp'),'-o',str(package/'osp_mission/_raycast.so')],check=True)
    subprocess.run(['g++','-std=c++17','-O1','-I/opt/ros/jazzy/include','-I/usr/include/eigen3',
        str(package/'tools/export_slam_graph_poses.cpp'),'-L/opt/ros/jazzy/lib','-Wl,-rpath,/opt/ros/jazzy/lib',
        '-lkartoSlamToolbox','-lboost_serialization','-lboost_thread','-lpthread',
        '-o',str(package/'tools/export_slam_graph_poses')],check=True)
    subprocess.run(['colcon','build','--symlink-install','--base-paths','src','--packages-select','osp_mission'],cwd=destination,check=True)
    unchanged=all(digest(package/name)==expected for name,expected in manifest['source_sha256'].items())
    binaries={name:dict(sha256=digest(package/name),matches_frozen_vm=digest(package/name)==expected)
              for name,expected in manifest['omitted_build_artifacts'].items()}
    report=dict(version='1.5.1',text_files=len(manifest['source_sha256']),text_source_unchanged=unchanged,
                binaries=binaries,ros_nodes_started=False,physical_validated=False)
    assert unchanged
    (destination/'snapshot_build.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('destination',type=Path)
    build(parser.parse_args().destination.resolve())
