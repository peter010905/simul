"""Shift the twin's desks to the desk-leg positions recurring in the 2026-11-27 physical SLAM maps.

usage: desk_update.py <base scene dir> <new scene dir> <deskfit_legs.json>
Desks with >= 2 legs whose physical occupancy share is >= 0.35 move by their mean leg offset;
the joined middle rows (table_middle_1-3, 4-6) move together by the row mean. Other desks stay.
"""
import json, re, shutil, sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
base, dest, fit = Path(sys.argv[1]), Path(sys.argv[2]), json.loads(Path(sys.argv[3]).read_text())
if dest.exists(): raise SystemExit(f"{dest} exists")
offset = {}
for desk, rows in fit["per_leg"].items():
    good = [r[1] for r in rows if r and r[0] >= .35]
    if len(good) >= 2: offset[desk] = np.mean(good, axis=0)
for row in (("table_middle_1", "table_middle_2", "table_middle_3"), ("table_middle_4", "table_middle_5", "table_middle_6")):
    known = [offset[d] for d in row if d in offset]
    for d in row:
        offset.pop(d, None)
        if known: offset[d] = np.mean(known, axis=0)
shutil.copytree(base, dest, symlinks=True)
sdf = dest / "world/osp_studio.sdf"
tree = ET.parse(sdf); moved = {}
for m in tree.getroot().iter("model"):
    if m.get("name") != "demo_furniture": continue
    for el in m.iter():
        if el.tag not in ("collision", "visual"): continue
        g = re.match(r"(table_[a-z]+_\d+)", el.get("name") or "")
        if not g or g.group(1) not in offset: continue
        pose = el.find("pose"); v = [float(x) for x in pose.text.split()]
        v[0] += offset[g.group(1)][0]; v[1] += offset[g.group(1)][1]
        pose.text = " ".join(f"{x:.6g}" for x in v); moved[g.group(1)] = moved.get(g.group(1), 0) + 1
tree.write(sdf, xml_declaration=True, encoding="utf-8")
(dest / "desk_update.json").write_text(json.dumps(dict(
    source="2026-11-27 physical SLAM maps (7 full-room runs), occupancy evidence at model legs",
    offsets_m={d: [round(float(o[0]), 3), round(float(o[1]), 3)] for d, o in sorted(offset.items())},
    elements_moved=moved, unchanged=sorted(set(fit["per_leg"]) - set(offset))), indent=1))
print(json.dumps({d: [round(float(o[0]), 2), round(float(o[1]), 2)] for d, o in sorted(offset.items())}))
print("unchanged:", sorted(set(fit["per_leg"]) - set(offset)))
