"""Copy frozen physical policies and change only SLAM maximum laser range."""
import hashlib,json,shutil
from pathlib import Path
import yaml
for name in ('v15240_box_safe_views_640_1124_ws','v15244_box_continuity_r1_640_1124_ws'):
 parent=Path.home()/name
 dest=Path.home()/(name.replace('_640_', '_lidar5_640_'))
 if dest.exists():
  raise FileExistsError(dest)
 dest.mkdir()
 for folder in ('src','frozen-config'):
  shutil.copytree(parent/folder,dest/folder,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
 for p in parent.glob('*_manifest.json'):shutil.copy2(p,dest/p.name)
 rel=Path('src/osp_mission/config/slam.yaml')
 old=(parent/rel).read_bytes()
 text=old.decode()
 assert text.count('max_laser_range: 12.0')==1
 (dest/rel).write_text(text.replace('max_laser_range: 12.0','max_laser_range: 5.0'))
 assert yaml.safe_load((dest/rel).read_text())['slam_toolbox']['ros__parameters']['max_laser_range']==5.
 (dest/'lidar5_manifest.json').write_text(json.dumps(dict(parent=name,range_m=5.,
  previous_range_m=12.,parent_config_sha256=hashlib.sha256(old).hexdigest()),indent=2))
 print(dest)
