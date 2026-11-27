"""Rebuild the twin's two long walls from the wall faces recurring in the physical SLAM maps.

usage: wall_rebuild.py <twin sdf in> <twin sdf out> <evidence.npz> <deskfit.json> <deskfit_legs.json> <png out>
The physical->twin transform is the one desk_fit.py found (walls by ICP, along-room offset by legs),
so the desks and the new walls stay consistent. Glass ends, chairs and desks are unchanged.
"""
import json, math, sys, copy
import numpy as np, cv2
import xml.etree.ElementTree as ET
from scipy import ndimage
SRC, DST, EV, FIT, LEGS, PNG = sys.argv[1:7]
tree = ET.parse(SRC); world = tree.getroot().find("world")
room = [m for m in world.findall("model") if m.get("name") == "slam_room"][0]
link = room.find("link")
def box_of(el):
    p = [float(v) for v in el.findtext("pose").split()]; s = [float(v) for v in el.find("geometry/box/size").text.split()]
    return p, s
old_walls = [el for el in link if (el.get("name") or "").startswith("wall") and el.tag == "collision"]
# twin wall sample points as desk_fit.py built them (<= 6 m wall segments) -> twin_centre
pts = []
for el in old_walls:
    (x, y, *_r), (sx, sy, _) = box_of(el)
    if max(sx, sy) > 6.: continue
    for u in np.linspace(-.5, .5, max(2, int(max(sx, sy) / .05))):
        for v in (-.5, .5):
            pts += [(x + u * sx, y + v * sy), (x + v * sx, y + u * sy)]
pts = np.array(pts); twin_centre = (pts.min(0) + pts.max(0)) / 2
fit = json.load(open(FIT))["physical_to_twin"]; slide = np.array(json.load(open(LEGS))["slide"])
R, t, flip = np.array(fit["R"]), np.array(fit["t"]), fit["flip"]
E = np.load(EV); share, known, res, W, H = E["share"], E["known"], float(E["res"]), float(E["W"]), float(E["H"])
mask = (share >= .5) & (known >= 3)
lab, n = ndimage.label(mask, structure=np.ones((3, 3)))
keep = [i + 1 for i, sl in enumerate(ndimage.find_objects(lab)) if sl and max(sl[0].stop - sl[0].start, sl[1].stop - sl[1].start) * res >= 1.0]
rr, cc = np.nonzero(np.isin(lab, keep))
q = np.stack(((cc + .5) * res - W / 2, (rr + .5) * res - H / 2), 1)
P = ((-q if flip else q) + twin_centre) @ R.T + t + slide          # physical wall cells in twin frame
glass = {el.get("name"): box_of(el) for el in link if (el.get("name") or "").startswith("glass") and el.tag == "collision"}
x_lo = glass["glass_west_collision"][0][0] + .15; x_hi = glass["glass_east_collision"][0][0] - .15
mid_y = float((pts[:, 1].min() + pts[:, 1].max()) / 2)
print("twin wall y %.2f..%.2f  physical cells y %.2f..%.2f x %.2f..%.2f  glass x %.2f..%.2f" % (pts[:, 1].min(), pts[:, 1].max(),
      P[:, 1].min(), P[:, 1].max(), P[:, 0].min(), P[:, 0].max(), x_lo, x_hi))
new_boxes, profiles = [], {}
for side in ("south", "north"):
    band = P[(P[:, 1] < mid_y - 3.) if side == "south" else (P[:, 1] > mid_y + 3.)]
    band = band[(band[:, 0] >= x_lo) & (band[:, 0] <= x_hi)]
    line = float(np.median(band[:, 1]))                                    # the wall's own line
    band = band[np.abs(band[:, 1] - line) <= .9]                           # steps <= ~0.6 m; end walls out
    xs = np.arange(x_lo, x_hi, .05); face = np.full(len(xs), np.nan)
    for i, x in enumerate(xs):
        col = band[np.abs(band[:, 0] - x) < .025, 1]
        if len(col): face[i] = col.max() if side == "south" else col.min()   # face toward the room
    ok = np.isfinite(face)
    face = np.interp(xs, xs[ok], face[ok])                                 # bridge gaps (doors, unseen)
    face = ndimage.median_filter(face, 7)
    segs = []; start = 0
    for i in range(1, len(xs) + 1):
        if i == len(xs) or abs(face[i] - np.median(face[start:i])) > .06:
            segs.append([xs[start], xs[i - 1] + .05, float(np.median(face[start:i]))]); start = i
    merged = []
    for s in segs:                                                        # fold slivers into a neighbour
        if merged and s[1] - s[0] < .15: merged[-1][1] = s[1]
        else: merged.append(s)
    profiles[side] = merged
    # Every segment reaches one common outer line, so steps leave no gap between neighbours.
    faces = [yf for _, _, yf in merged]
    outer = (min(faces) - .30) if side == "south" else (max(faces) + .30)
    for x0, x1, yf in merged:
        th = abs(yf - outer); yc = (yf + outer) / 2
        new_boxes.append(((x0 + x1) / 2, yc, x1 - x0, th))
# measure: physical face cells vs twin walls, before/after
def surf_dist(boxes, P):
    d = np.full(len(P), np.inf)
    for x, y, sx, sy in boxes:
        dx = np.maximum(np.abs(P[:, 0] - x) - sx / 2, 0); dy = np.maximum(np.abs(P[:, 1] - y) - sy / 2, 0)
        d = np.minimum(d, np.hypot(dx, dy))
    return d
long_band = np.concatenate([P[(sel := ((P[:, 1] < mid_y - 3.) if side == "south" else (P[:, 1] > mid_y + 3.))
                               & (P[:, 0] >= x_lo) & (P[:, 0] <= x_hi))] for side in ("south", "north")])
long_band = long_band[np.array([abs(p[1] - np.median(long_band[np.sign(long_band[:, 1] - mid_y) == np.sign(p[1] - mid_y), 1])) <= .9
                                for p in long_band])]
old = [(box_of(el)[0][0], box_of(el)[0][1], box_of(el)[1][0], box_of(el)[1][1]) for el in old_walls]
b_before, b_after = surf_dist(old, long_band), surf_dist(new_boxes, long_band)
print("physical long-wall cells %d: distance to twin walls  before median %.3f m, p90 %.3f m | after median %.3f m, p90 %.3f m"
      % (len(long_band), np.median(b_before), np.percentile(b_before, 90), np.median(b_after), np.percentile(b_after, 90)))
print("segments:", {k: len(v) for k, v in profiles.items()})
# rewrite the SDF: drop old walls (collision+visual), add new ones, drop wall-side objects buried in new walls
material = copy.deepcopy([el for el in link if (el.get("name") or "").startswith("wall") and el.tag == "visual"][0].find("material"))
for el in [el for el in link if (el.get("name") or "").startswith("wall")]:
    link.remove(el)
buried = 0
for el in [el for el in link if (el.get("name") or "").startswith("object") and el.tag == "collision"]:
    (x, y, *_), _ = box_of(el)
    if any(abs(x - bx) <= sx / 2 + .02 and abs(y - by) <= sy / 2 + .02 for bx, by, sx, sy in new_boxes):
        stem = el.get("name").rsplit("_", 1)[0]
        for e2 in [e for e in link if e.get("name") in (stem + "_collision", stem + "_visual")]: link.remove(e2)
        buried += 1
for k, (x, y, sx, sy) in enumerate(new_boxes):
    for tag in ("collision", "visual"):
        e = ET.SubElement(link, tag, name=f"wall_{k}_{tag}")
        ET.SubElement(e, "pose").text = f"{x:.4f} {y:.4f} 1.2500 0 0 0"
        g = ET.SubElement(ET.SubElement(e, "geometry"), "box"); ET.SubElement(g, "size").text = f"{sx:.4f} {sy:.4f} 2.5000"
        if tag == "visual": e.append(copy.deepcopy(material))
tree.write(DST, xml_declaration=True, encoding="utf-8")
print("new wall boxes", len(new_boxes), "| wall-side objects removed (inside new walls):", buried)
# picture
lo = np.array([x_lo - 1, mid_y - 8]); S = 40
img = np.full((int(16 * S), int((x_hi - x_lo + 2) * S), 3), 255, np.uint8)
px = lambda x, y: (int((x - lo[0]) * S), int((lo[1] + 16 - y) * S))
for x, y, sx, sy in old: cv2.rectangle(img, px(x - sx / 2, y + sy / 2), px(x + sx / 2, y - sy / 2), (200, 200, 200), -1)
for p in long_band[::2]: cv2.circle(img, px(*p), 1, (0, 0, 255), -1)
for x, y, sx, sy in new_boxes: cv2.rectangle(img, px(x - sx / 2, y + sy / 2), px(x + sx / 2, y - sy / 2), (0, 140, 0), 1)
cv2.putText(img, "grey = old twin walls   red = physical SLAM wall cells   green = new walls", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 0), 1)
cv2.imwrite(PNG, img)
