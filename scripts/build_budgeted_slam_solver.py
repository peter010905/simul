"""Build an isolated, explicitly selected Ceres plugin with a thread budget.

Never replaces the installed SLAM Toolbox library or a running native source.
Upstream v2.8.5 source and license are retained with checked SHA256 digests.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import urllib.request

SOURCES={
    'solvers/ceres_solver.cpp':'e55f6442834e9dc450b049570f34399d0654f86df956597af6eab700fcb7c118',
    'solvers/ceres_solver.hpp':'b99c5d7f012827c396d6ac5377eddf4c3c56b60dc91c624d012682aa6c0adc6a',
    'solvers/ceres_utils.h':'ed56f5d2a9843b3029ed85fa46db44de024e3c2a16036774a3c4dc73d0fde08a',
    'LICENSE':'20c17d8b8c48a600800dfd14f95d5cb9ff47066a9641ddeab48dc54aec96e331'}
BASE='https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/2.8.5/'


def build(output):
    output.mkdir(parents=True,exist_ok=False)
    manifest=dict(upstream_version='2.8.5',upstream_source=BASE,sha256=SOURCES,
                  installed_system_files_changed=False,production_promoted=False)
    for name,expected in SOURCES.items():
        data=urllib.request.urlopen(BASE+name,timeout=30).read()
        if hashlib.sha256(data).hexdigest()!=expected:raise ValueError('Upstream source fingerprint changed')
        path=output/'upstream'/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
        path=output/'source'/name;path.parent.mkdir(parents=True,exist_ok=True)
        code=data.decode()
        if name.endswith(('ceres_solver.cpp','ceres_solver.hpp')):
            code=code.replace('solver_plugins','osp_solver_budget')
        if name.endswith('ceres_solver.cpp'):
            before='  options_.num_threads = 50;'
            if code.count(before)!=1:raise ValueError('Unexpected solver thread assignment')
            code=code.replace(before,
                '  if (!node->has_parameter("osp_solver_threads")) node->declare_parameter("osp_solver_threads", 2);\n'
                '  const int threads = node->get_parameter("osp_solver_threads").as_int();\n'
                '  if (threads < 1 || threads > 12) throw std::invalid_argument("osp_solver_threads must be 1..12");\n'
                '  options_.num_threads = threads;\n'
                '  RCLCPP_INFO(logger_, "Ceres CPU budget: %d threads", threads);')
        path.write_text(code)
    prefix=output/'install';lib=prefix/'lib';lib.mkdir(parents=True)
    include=Path('/opt/ros/jazzy/include')
    cmd=['g++','-std=c++17','-O2','-fPIC','-shared','-I'+str(output/'source'),
         '-I'+str(include),'-I/usr/include/eigen3']+['-I'+str(p) for p in sorted(include.iterdir()) if p.is_dir()]
    cmd += [str(output/'source/solvers/ceres_solver.cpp'),'-L/opt/ros/jazzy/lib','-Wl,-rpath,/opt/ros/jazzy/lib',
        '-lceres','-lkartoSlamToolbox','-lboost_thread','-lboost_serialization','-lclass_loader','-lrclcpp',
        '-lrclcpp_lifecycle','-lrcutils','-lconsole_bridge','-lglog','-lpthread','-o',str(lib/'libosp_solver_budget.so')]
    with (output/'build.log').open('w') as stream:subprocess.run(cmd,stdout=stream,stderr=stream,check=True)
    package=prefix/'share/osp_solver_budget';package.mkdir(parents=True)
    (package/'package.xml').write_text('<package format="3"><name>osp_solver_budget</name><version>0.1.0</version><description>Isolated CPU-budget experiment for SLAM Toolbox 2.8.5 Ceres solver</description><maintainer email="local@example.invalid">OSP local experiment</maintainer><license>LGPL-2.1</license></package>\n')
    (package/'solver_plugins.xml').write_text('<library path="osp_solver_budget"><class name="osp_solver_budget::CeresSolver" type="osp_solver_budget::CeresSolver" base_class_type="karto::ScanSolver"><description>Isolated Ceres solver with explicit CPU thread budget</description></class></library>\n')
    (package/'LICENSE').write_bytes((output/'upstream/LICENSE').read_bytes())
    index=prefix/'share/ament_index/resource_index'
    for kind,value in [('packages',''),('slam_toolbox__pluginlib__plugin','share/osp_solver_budget/solver_plugins.xml\n')]:
        path=index/kind/'osp_solver_budget';path.parent.mkdir(parents=True,exist_ok=True);path.write_text(value)
    manifest.update(command=cmd,plugin='osp_solver_budget::CeresSolver',prefix=str(prefix),
                    executable_sha256=hashlib.sha256((lib/'libosp_solver_budget.so').read_bytes()).hexdigest())
    (output/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    build(parser.parse_args().output.resolve())
