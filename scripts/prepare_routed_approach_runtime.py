"""Stage known-free navigation routes to confirmed marker viewpoints."""
import argparse
from pathlib import Path
import shutil

from prepare_approach_witness_runtime import replace_once

ROOT=Path(__file__).resolve().parents[1]


def prepare(source,destination):
    if destination.exists():raise FileExistsError('Use a new immutable candidate')
    package=destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission',package,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    path=package/'osp_mission/marker_approach.py';code=path.read_text()
    code=replace_once(code,'from osp_mission.policy import wrap',
        'from osp_mission.policy import wrap, graph\nfrom osp_mission.escape_connectivity import connect_start\nfrom scipy.sparse.csgraph import dijkstra')
    code=replace_once(code,"        safe = np.where((data == 0) & (distance_transform_edt(data <= 50)*resolution >= .32), 0, 100)\n        options = []",
        "        goal_safe = (data == 0) & (distance_transform_edt(data == 0)*resolution >= .32)\n"
        "        cell = np.floor((robot-origin)/resolution).astype(int)\n"
        "        if not (0 <= cell[1] < data.shape[0] and 0 <= cell[0] < data.shape[1]):return None\n"
        "        route_safe = connect_start(data,resolution,(cell[1],cell[0]),goal_safe)\n"
        "        route_distance = np.full(data.shape,np.inf)\n"
        "        if route_safe[cell[1],cell[0]]:\n"
        "            route_distance = dijkstra(graph(route_safe,resolution),directed=False,\n"
        "                indices=cell[1]*data.shape[1]+cell[0]).reshape(data.shape)\n"
        "        options = []")
    code=replace_once(code,"                    a, b = (robot-origin)/resolution, (position-origin)/resolution\n"
        "                    if not visible(safe, a[1], a[0], np.array([b[1]]), np.array([b[0]]))[0]:\n"
        "                        continue\n                    distance = float(np.linalg.norm(position-robot))",
        "                    cell = np.floor((position-origin)/resolution).astype(int)\n"
        "                    if not (0 <= cell[1] < data.shape[0] and 0 <= cell[0] < data.shape[1]):continue\n"
        "                    if not goal_safe[cell[1],cell[0]] or not np.isfinite(route_distance[cell[1],cell[0]]):continue\n"
        "                    distance = float(route_distance[cell[1],cell[0]])")
    path.write_text(code)
    shutil.copy2(ROOT/'scripts/test_routed_approach_runtime.py',package/'test/test_routed_approach_runtime.py')
    return destination


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('source','destination'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();print(prepare(args.source.resolve(),args.destination.resolve()))
