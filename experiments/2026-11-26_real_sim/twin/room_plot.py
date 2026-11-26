"""Top view of a room_scene.py world over the registered SLAM: python3 room_plot.py <root> <scene> <out.png>."""
import json, sys, xml.etree.ElementTree as ET
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
root, scene, out = sys.argv[1:4]
b = json.load(open(f"{root}/registered-slam.json"))
img = plt.imread(f"{root}/registered-slam.png")
fig, ax = plt.subplots(figsize=(15, 8.5), constrained_layout=True)
ax.imshow(img, cmap="gray", vmin=0, vmax=1, extent=(b["x"], b["x"]+b["width"], -b["y"]-b["height"], -b["y"]), alpha=.55)
w = ET.parse(f"{scene}/world/osp_studio.sdf").find("world")
colours = {"wall": "#8a6d3b", "glass": "#00a0ff", "chair": "#d62728", "object": "#9467bd", "table": "#2ca02c"}
for m in w.findall("model"):
    mp = [float(v) for v in (m.findtext("pose") or "0 0 0 0 0 0").split()]
    for el in m.iter("collision"):
        size = el.find("geometry/box/size")
        if size is None: continue
        sx, sy, _ = map(float, size.text.split())
        x, y = [float(v) for v in (el.findtext("pose") or "0 0").split()[:2]]
        if m.get("name", "").startswith("cable_"):
            import numpy as np
            L = float(m.find("link/collision/geometry/cylinder/length").text)
            dx, dy = np.cos(mp[5]) * L / 2, np.sin(mp[5]) * L / 2
            ax.plot([mp[0] - dx, mp[0] + dx], [mp[1] - dy, mp[1] + dy], "-", color="#111111", lw=1.5)
            continue
        if m.get("name", "").startswith("wallobj_"):
            ax.plot(mp[0], mp[1], "s", color="#2b6cb0", ms=9)
            continue
        if m.get("name", "").startswith("debris_"):
            ax.plot(mp[0], mp[1], "x", color="#000000", ms=7, mew=2)
            continue
        if m.get("name", "").startswith("movable_"):
            import numpy as np
            c, sn = np.cos(mp[5]), np.sin(mp[5])
            ax.add_patch(Rectangle((mp[0] - c*sx/2 + sn*sy/2, mp[1] - sn*sx/2 - c*sy/2), sx, sy,
                                   angle=np.degrees(mp[5]), color="#e6a03c", alpha=.9, lw=0))
            continue
        if m.get("name") == "demo_furniture":
            kind = "table"
            if sx > 1 or sy > 1: continue
        else:
            n = el.get("name"); kind = next((k for k in colours if n.startswith(k)), None)
            if kind is None: continue
        ax.add_patch(Rectangle((mp[0]+x-sx/2, mp[1]+y-sy/2), sx, sy, color=colours[kind], alpha=.8 if kind!="glass" else 1, lw=0))
for t in json.load(open(f"{scene}/scene.json"))["targets"]:
    ax.plot(t["x"], t["y"], "*", ms=16, color="orange"); ax.text(t["x"]+.2, t["y"]+.2, f"ID {t['id']}")
s = json.load(open(f"{scene}/reference.json"))["start"]; ax.plot(s[0], s[1], "go", ms=10); ax.arrow(s[0], s[1], .8*np.cos(s[2]), .8*np.sin(s[2]), color="g", width=.06)
ax.set_xlim(-14, 11); ax.set_ylim(-7.3, 7.3); ax.set_aspect("equal")
ax.set_title("scene16-room: walls (brown), glass (blue, collision only), chair posts (red), objects (purple), desk legs (green)")
fig.savefig(out, dpi=90)
