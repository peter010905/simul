from pathlib import Path
import math,yaml
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Circle,Rectangle
from osp_mission.scenarios import fixtures
from osp_mission.world import generate
cfg=yaml.safe_load(Path('src/osp_mission/config/robust_demo.yaml').read_text())
generate('src/osp_mission/config/robust_demo.yaml','training_robust/demo_world')
fig,ax=plt.subplots(figsize=(12,7));ax.add_patch(Rectangle((0,0),20.15,11.35,fc='#f5f7fa',ec='#243b55',lw=3))
for j,o in enumerate(fixtures()+cfg['obstacles']):
 if o['type']=='cylinder':patch=Circle((o['x'],o['y']),o['radius'],fc='#879eaf',ec='#34495e')
 else:
  a=o.get('yaw',0.);rotation=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
  corners=np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*[o['sx']/2,o['sy']/2]
  patch=Polygon(corners@rotation.T+[o['x'],o['y']],fc='#879eaf',ec='#34495e')
 ax.add_patch(patch)
for t in cfg['targets']:
 ax.scatter(t['x'],t['y'],marker='*',s=180,c='#168768',zorder=6)
 ax.arrow(t['x'],t['y'],.7*math.cos(t['yaw']),.7*math.sin(t['yaw']),width=.035,color='#168768',zorder=5)
 ax.annotate(f"ID {t['id']}",(t['x'],t['y']),xytext=(9,12),textcoords='offset points',fontsize=10)
x,y,a=cfg['start'];ax.scatter(x,y,s=130,marker='^',c='#e99b2d');ax.arrow(x,y,.7*math.cos(a),.7*math.sin(a),width=.035,color='#e99b2d');ax.annotate('START',(x,y),xytext=(8,-17),textcoords='offset points')
ax.set(xlim=(-.5,20.7),ylim=(-.6,11.9),aspect='equal',xlabel='World x (m)',ylabel='World y (m)',title='Dense default: 12 obstacles / 3 markers on hidden obstacle faces')
ax.grid(alpha=.15);fig.text(.5,.035,'Layout preview only. Marker ground truth is never provided to the navigation policy.',ha='center')
fig.tight_layout(rect=(0,.06,1,1));fig.savefig('training_robust/robust_preview.png',dpi=160)
