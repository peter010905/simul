"""Build read-only SLAM inspection/replay tools outside native runtime stages."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def build(destination, ros_prefix=Path('/opt/ros/jazzy')):
    destination.mkdir(parents=True, exist_ok=False)
    include = ros_prefix/'include'
    common = ['g++','-std=c++17','-O1','-I'+str(include),'-I/usr/include/eigen3']
    # Jazzy installs dependency headers in per-package include directories.
    common += ['-I'+str(p) for p in sorted(include.iterdir()) if p.is_dir()]
    libraries = ['-L'+str(ros_prefix/'lib'),'-Wl,-rpath,'+str(ros_prefix/'lib'),
                 '-lkartoSlamToolbox','-lboost_serialization','-lboost_thread','-lpthread']
    manifest = dict(ros_prefix=str(ros_prefix),native_runtime_modified=False,programs=[])
    for name in ('export_slam_graph_edges','replay_karto_component'):
        source = ROOT/'training_robust'/(name+'.cpp')
        extra = ['-lclass_loader','-lament_index_cpp','-lrclcpp_lifecycle','-lrclcpp',
                 '-lrcutils','-lrcpputils','-lconsole_bridge','-ltinyxml2'] if name.startswith('replay_') else []
        command = common+[str(source)]+libraries+extra+['-o',str(destination/name)]
        with (destination/(name+'.build.log')).open('w') as stream:
            subprocess.run(command,stdout=stream,stderr=stream,check=True)
        copy = destination/source.name
        copy.write_bytes(source.read_bytes())
        digest = lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
        manifest['programs'].append(dict(name=name,source_snapshot=str(copy),source_sha256=digest(copy),
            executable_sha256=digest(destination/name),command=command))
    (destination/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args();build(args.output.resolve())
