import json,math
from pathlib import Path
from types import MethodType
import numpy as np
import rosbag2_py as bag
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from osp_mission.r32_drive import grid_from_message
from osp_mission import _r32_navigation_core as nav
from osp_mission.center_wall import CenterWall
from osp_mission.box_access import box_routes
from osp_mission.box_views import odom_point
from osp_mission.box_tour import short_tour
run=Path.home()/'simul/run_logs/robot/20261125T174857-2f02f6e167'
rows=[json.loads(l) for l in (run/'artifacts/center_wall.jsonl').read_text().splitlines()]
start=next(x['wall_epoch_s'] for x in map(json.loads,(run/'artifacts/startup.jsonl').read_text().splitlines()) if x['event']=='mission_started')
reader=bag.SequentialReader();reader.open(bag.StorageOptions(uri=str(run/'rosbag'),storage_id='mcap'),bag.ConverterOptions('',''))
names={'/odom','/tf','/local_costmap/costmap','/global_costmap/costmap'}
types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in names};reader.set_filter(bag.StorageFilter(topics=list(types)))
def stream():
 while reader.has_next():
  n,r,t=reader.read_next(); yield n,deserialize_message(r,types[n]),t/1e9-start
it=iter(stream());current=next(it,None);costs={};transform=None;wheel=None;results=[]
for i,row in enumerate(rows):
 if row['batch_box'] is None:continue
 f=run/'artifacts/policy_decisions'/f'{i:04d}.npz'
 # Policy audits can include empty planning attempts. Match elapsed rather than index.
 found=None
 for path in (run/'artifacts/policy_decisions').glob('*npz'):
  a=np.load(path)
  if abs(float(a['elapsed'])-row['elapsed'])<.1:found=path;break
 if found is None:continue
 a=np.load(found);elapsed=float(a['elapsed'])
 while current and current[2]<=elapsed:
  name,m,t=current
  if name=='/tf':
   for tr in m.transforms:
    if (tr.header.frame_id,tr.child_frame_id)==('map','odom'):
     q=tr.transform.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));x,y=tr.transform.translation.x,tr.transform.translation.y
     c,s=math.cos(yaw),math.sin(yaw);transform=(-c*x-s*y,s*x-c*y,-yaw)
  elif name=='/odom':
   q=m.pose.pose.orientation;p=m.pose.pose.position
   wheel=(t,p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
  else:costs[name]=(m.header.frame_id,grid_from_message(m))
  current=next(it,None)
 transform_source='recorded_tf'
 if transform is None and wheel and abs(wheel[0]-elapsed)<.15:
  angle=wheel[3]-float(a['heading']);c,s=math.cos(angle),math.sin(angle);x,y=a['robot']
  transform=(wheel[1]-c*x+s*y,wheel[2]-s*x-c*y,angle)
  transform_source='paired_audit_and_odom'
 if transform is None or '/global_costmap/costmap' not in costs:continue
 grid=nav.Grid(a['data'],float(a['resolution']),(*a['origin'],0.));pose=(*a['robot'],float(a['heading']))
 center=CenterWall()
 center.batch.update([row['batch_box']],[],odom_point(pose[:2],transform))
 center.local_box_cost=costs.get('/local_costmap/costmap')
 captured=[]
 def capture(ranked,*args):
  captured.extend(ranked)
  return None
 center.continuity.choose=capture
 center.decide(grid,costs['/global_costmap/costmap'][1],pose,transform,a['viewed'],elapsed,600.,lambda _:False,[])
 candidates=[(w,g) for w,g in captured if g.get('batch_box') is not None and math.dist(g['batch_box'],row['batch_box'])<.4]
 safe,_=grid.reachable(pose[:2],.32)
 cost=costs['/global_costmap/costmap'][1]
 joint,_=box_routes(grid,cost,nav.connected_nav_cells(cost,pose),safe,pose,transform,center.local_box_cost)
 choice=short_tour(candidates,grid,joint,55.)
 results.append(dict(snapshot=found.name,elapsed=round(elapsed,2),recorded_box=row['batch_box'],
                     regenerated_views=len(candidates),local_cost_used=center.local_box_cost is not None,transform_source=transform_source,
                     tour_faces=choice.get('box_tour_faces') if choice else 0,
                     tour_walk_m=choice.get('box_tour_walk_m') if choice else None))
 if transform_source=='paired_audit_and_odom':transform=None
print(json.dumps(dict(scope='Recorded SLAM and costmap geometry; only the historically selected box center is seeded. Reconstructed generic face candidates. No actual Nav2 planning or new driving.',checks=results),indent=2))
