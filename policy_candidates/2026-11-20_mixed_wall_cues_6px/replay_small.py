import argparse,bisect,csv,json,math,sys,time
from pathlib import Path
import cv2
import numpy as np
from osp_mission.small_wall_cues import candidates
from osp_mission.wall_bearings import WallBearings,wrap
p=Path(sys.argv[1]); cv2.setNumThreads(1)
scene=json.loads((p/'scene.json').read_text())
markers=scene['targets']; wall=next(m for m in markers if m['attached_to'].startswith('wall'))
head=[tuple(float(r[k]) for k in ('elapsed_sim_s','x','y','yaw_rad')) for r in csv.DictReader(open(p/'headings.csv'))]; ts=[r[0] for r in head]
frames=list(csv.DictReader(open(p/'frames.csv')));dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
tracks=WallBearings(); episodes=[];seen=set(); took=[];count=match=direct=0; bandframes=0; rescued=0; hits_before=None
for f in frames:
 t=float(f['elapsed_sim_s']); img=cv2.imread(str(p/'camera'/f"{int(f['frame']):05d}.jpg"),cv2.IMREAD_GRAYSCALE)
 q,ids,rej=cv2.aruco.detectMarkers(img,dictionary); pts=[]
 if ids is not None:
  pts=[(c.reshape(4,2).mean(axis=0),int(i)) for c,i in zip(q,ids.ravel()) if 6<=np.linalg.norm(c.reshape(4,2)-np.roll(c.reshape(4,2),1,axis=0),axis=1).min()<10.]
 start=time.perf_counter();hits=candidates(img,q,rej);took.append((time.perf_counter()-start)*1000)
 direct+=sum(i==wall['id'] for a,i in pts)
 wf=[a for a,i in pts if i==wall['id']]
 bandframes+=bool(wf)
 rescued+=bool(wf) and any(math.dist((u,v),a)<4 for u,v,_,_ in hits for a in wf)
 i=min(len(head)-1,bisect.bisect_left(ts,t));_,x,y,h=head[i]
 for u,v,s,named in hits:
  count+=1
  actual=next((i for a,i in pts if math.dist(a,(u,v))<=4),None)
  if actual==wall['id']:match+=1
  a=h-math.atan2(u-320,443.53)
  tracks.observe(t,t,(x,y),a,s)
 for tr in tracks.tracks:
  if not tr.stable():continue
  # Identify a temporal episode once even if its sliding window subsequently changes.
  if getattr(tr,'evaluated',False):continue
  tr.evaluated=True
  _,origin,bearing,_=tr.samples[-1]
  diffs=[(abs(wrap(math.atan2(m['y']-origin[1],m['x']-origin[0])-bearing)),m['id']) for m in markers]
  residual,near=min(diffs)
  episodes.append(dict(t=round(t,2),nearest=near,angular_err_deg=round(math.degrees(residual),2)))
  if near==wall['id'] and residual<math.radians(3) and hits_before is None: hits_before=t
res=dict(run=p.name,frames=len(frames),proposals=count,wall_id=wall['id'],weak_decoded_wall_frames=bandframes,weak_filter_matched_wall_frames=rescued,
 episodes=len(episodes),episodes_outside_3deg_all_markers=sum(e['angular_err_deg']>3 for e in episodes),
 episodes_wall_bearing=sum(e['angular_err_deg']<=3 and e['nearest']==wall['id'] for e in episodes),
 first_wall_bearing_s=hits_before,extra_filter_mean_ms=round(float(np.mean(took)),3),extra_filter_p95_ms=round(float(np.percentile(took,95)),3),
 details=episodes)
print(json.dumps(res),flush=True)
