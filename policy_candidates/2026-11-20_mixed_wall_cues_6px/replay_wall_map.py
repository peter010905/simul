"""Counterfactual wall ranking on historical maps. Substitutes SLAM free cells for unavailable Nav2 costs."""
from pathlib import Path
from types import SimpleNamespace
import bisect,csv,json,math,sys
import cv2,numpy as np
from osp_mission._r32_navigation_core import Grid
from osp_mission.center_wall import CenterWall
from osp_mission.small_wall_cues import candidates
from osp_mission.wall_bearings import WallBearings

p=Path(sys.argv[1])
cv2.setNumThreads(1)
poses=[tuple(float(r[k]) for k in ('elapsed_sim_s','x','y','yaw_rad')) for r in csv.DictReader(open(p/'headings.csv'))];ts=[r[0] for r in poses]
frames=list(csv.DictReader(open(p/'frames.csv'))); fi=0
tracker=WallBearings(); dic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
outputs=[]
for snapshot in sorted((p/'native/policy_decisions').glob('*.npz'))[::3]:
 z=np.load(snapshot);t=float(z['elapsed'])
 while fi<len(frames) and float(frames[fi]['elapsed_sim_s'])<=t:
  row=frames[fi];fi+=1;ft=float(row['elapsed_sim_s'])
  image=cv2.imread(str(p/'camera'/f"{int(row['frame']):05d}.jpg"),0)
  q,ids,rejected=cv2.aruco.detectMarkers(image,dic)
  _,x,y,h=poses[min(len(poses)-1,bisect.bisect_left(ts,ft))]
  for u,v,size,_ in candidates(image,q,rejected):
   tracker.observe(ft,ft,(x,y),h-math.atan2(u-320,443.53),size)
 robot=np.r_[z['robot'][:2],float(z['heading'])]; data=z['data']; orig=z['origin'].reshape(-1);grid=Grid(data,float(z['resolution']),(*orig[:2],0.))
 _,x,y,h=poses[min(len(poses)-1,bisect.bisect_left(ts,t))]; a=h-robot[2]; c,s=math.cos(a),math.sin(a)
 tr=(x-c*robot[0]+s*robot[1],y-s*robot[0]-c*robot[1],a)
 row=dict(t=round(t,2),stable=sum(k.stable() for k in tracker.tracks))
 def audit(walls,grid,transform,known,now,left):
  row['walls']=len(walls)
  v=tracker.choose(walls,grid,transform,known,now,left)
  if v:row['wall_point_world']=v['surface_odom']
  return v
 schedule=CenterWall(); schedule.weak=SimpleNamespace(choose=audit);schedule.weak_clock=t
 schedule.decide(grid,grid,robot,tr,z['viewed'],t,600,lambda goal:False)
 outputs.append(row)
print(json.dumps(outputs))
