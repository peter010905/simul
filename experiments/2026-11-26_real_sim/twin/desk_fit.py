"""Fit the twin's 16 desks to recurring desk legs from the physical SLAM maps (evaluator side).

usage: desk_fit.py <twin world sdf> <maps dir> <legs2.json> <reference map name> <out dir>
(after desk_legs_from_slam.py has written legs2.json and evidence.npz to <out dir>)
"""
import json, math, re, sys, collections
import numpy as np, cv2
import xml.etree.ElementTree as ET
from scipy.spatial import cKDTree
sys.argv, args = sys.argv[:1], sys.argv[1:]
from pathlib import Path
# Reuse the registration helpers of desk_legs_from_slam.py (its script part is not executed).
exec(Path(__file__).with_name("desk_legs_from_slam.py").read_text().split("ref = None; frames = {}")[0]
     .replace("D, OUT = sys.argv[1], sys.argv[2]\nnames = sys.argv[3].split(\",\")", ""))
SDF, D, LEGS, REFMAP, OUT = args
world = ET.parse(SDF).find("world")

def pose_of(el):
    return [float(v) for v in (el.findtext("pose") or "0 0 0 0 0 0").split()]

wall_pts, legs_model = [], collections.defaultdict(list)
for m in world.findall("model"):
    mp = pose_of(m); name = m.get("name")
    for c in m.iter("collision"):
        size = c.find("geometry/box/size")
        if size is None: continue
        sx, sy, _ = map(float, size.text.split()); p = pose_of(c)
        x, y, yaw = mp[0] + p[0], mp[1] + p[1], p[5] + mp[5]
        cn = c.get("name") or ""
        if name == "slam_room" and cn.startswith("wall") and max(sx, sy) <= 6.:
            for u in np.linspace(-.5, .5, max(2, int(max(sx, sy) / .05))):
                for v in (-.5, .5):
                    for a, b in ((u * sx, v * sy), (v * sx, u * sy)):
                        wall_pts.append((x + a * math.cos(yaw) - b * math.sin(yaw), y + a * math.sin(yaw) + b * math.cos(yaw)))
        if name == "demo_furniture" and "_leg" in cn:
            desk = re.match(r"(table_[a-z]+_\d+)", cn).group(1)
            legs_model[desk].append((x, y))
wall_pts = np.array(wall_pts)
print("twin desks", len(legs_model), "legs", sum(len(v) for v in legs_model.values()), "wall pts", len(wall_pts))

# reference physical map walls in its room frame, as legs.py defined it
d, res, o = load(REFMAP)
centre, ang, size = room_frame(d, res, o)
walls = big_points(d, res, o)
twin_tree = cKDTree(wall_pts)
twin_centre = (wall_pts.min(0) + wall_pts.max(0)) / 2
L = json.load(open(LEGS))["legs"]
P0 = np.array([(x, y) for x, y, *_ in L])
model_all = np.array([p for v in legs_model.values() for p in v])
best = None
for flip in (0., math.pi):
    src = to_room(walls, centre, ang + flip) + twin_centre
    R, t, err, frac = icp(src[::2], twin_tree)
    P = ((-P0 if flip else P0) + twin_centre) @ R.T + t
    dist, _ = cKDTree(P).query(model_all, distance_upper_bound=.4)
    matched = int(np.isfinite(dist).sum())
    print("flip %d: wall inlier %.2f mean %.3f, model legs with an observed leg within 0.4 m: %d" % (round(math.degrees(flip)), frac, err, matched))
    if best is None or matched > best[5]:
        best = (flip, R, t, frac, err, matched)
flip, R, t, frac, err, _ = best
print("physical->twin walls: inlier share %.2f, mean dist %.3f m, rotation %.2f deg" % (frac, err, math.degrees(math.atan2(R[1, 0], R[0, 0]))))
P = ((-P0 if flip else P0) + twin_centre) @ R.T + t
# The long walls fix the cross-room position and the angle, but not the position along the room
# (its ends are glass and chair bands in the twin). Slide along the long axis (and +-0.3 m across)
# to the offset where most model legs have an observed leg within 0.15 m.
model_tree = cKDTree(model_all)
slide = max(((dx, dy) for dx in np.arange(-3., 3.001, .05) for dy in np.arange(-.3, .301, .05)),
            key=lambda o: (int(np.isfinite(model_tree.query(P + o, distance_upper_bound=.15)[0]).sum()), -abs(o[0]) - abs(o[1])))
hits = int(np.isfinite(model_tree.query(P + slide, distance_upper_bound=.15)[0]).sum())
print("slide along the room %.2f m, across %.2f m: observed legs within 0.15 m of a model leg: %d of %d" % (slide[0], slide[1], hits, len(P)))
P = P + slide
obs = cKDTree(P)
report = {}
for desk, mlegs in sorted(legs_model.items()):
    M = np.array(mlegs); found = []
    for m in M:
        dist, j = obs.query(m, distance_upper_bound=.30)
        if np.isfinite(dist): found.append((m, P[j]))
    if len(found) >= 2:
        shift = np.mean([b - a for a, b in found], axis=0)
        resid = float(np.mean([np.linalg.norm(b - a - shift) for a, b in found]))
        report[desk] = dict(legs_matched=len(found), shift=[round(float(shift[0]), 3), round(float(shift[1]), 3)], residual=round(resid, 3))
    else:
        report[desk] = dict(legs_matched=len(found))
for k, v in report.items(): print(k, v)
json.dump(dict(physical_to_twin=dict(flip=flip, R=R.tolist(), t=t.tolist(), inlier=frac, mean=err), desks=report,
               observed_legs_twin=P.tolist()), open(OUT + "/deskfit.json", "w"), indent=1)
# picture in twin frame
res_i = .05; lo = wall_pts.min(0) - .5; hi = wall_pts.max(0) + .5
img = np.full((int((hi[1] - lo[1]) / res_i), int((hi[0] - lo[0]) / res_i), 3), 255, np.uint8)
px = lambda p: (int((p[0] - lo[0]) / res_i), int((hi[1] - p[1]) / res_i))
for p in wall_pts[::3]: cv2.circle(img, px(p), 1, (120, 120, 120), -1)
for p in (to_room(walls, centre, ang + flip) + twin_centre) @ R.T + t:
    cv2.circle(img, px(p), 0, (200, 200, 255), -1)
for desk, mlegs in legs_model.items():
    for m in mlegs: cv2.rectangle(img, (px(m)[0] - 3, px(m)[1] - 3), (px(m)[0] + 3, px(m)[1] + 3), (0, 150, 0), 1)
for p in P: cv2.circle(img, px(p), 3, (0, 0, 255), -1)
cv2.imwrite(OUT + "/deskfit.png", cv2.resize(img, None, fx=1.5, fy=1.5))

# --- per model leg: strongest physical evidence within 0.3 m (share of observing maps occupied)
E = np.load(OUT + "/evidence.npz"); share, known, eres, EW, EH = E["share"], E["known"], float(E["res"]), float(E["W"]), float(E["H"])
def to_ref(p):
    q = ((np.asarray(p) - t - np.array(slide)) @ R) - twin_centre
    return -q if flip else q
per_leg = {}
for desk, mlegs in sorted(legs_model.items()):
    rows = []
    for m in mlegs:
        q = to_ref(m); c0, r0 = int((q[0] + EW / 2) / eres), int((q[1] + EH / 2) / eres); w = int(.3 / eres)
        win = share[max(0, r0 - w):r0 + w + 1, max(0, c0 - w):c0 + w + 1]
        kn = known[max(0, r0 - w):r0 + w + 1, max(0, c0 - w):c0 + w + 1]
        if win.size == 0 or kn.max() < 3:
            rows.append(None); continue
        rr, cc = np.unravel_index(np.argmax(win), win.shape)
        peak = float(win[rr, cc])
        pq = np.array([(max(0, c0 - w) + cc + .5) * eres - EW / 2, (max(0, r0 - w) + rr + .5) * eres - EH / 2])
        pt = ((-pq if flip else pq) + twin_centre) @ R.T + t + np.array(slide)
        rows.append((peak, (pt - np.asarray(m)).tolist()))
    per_leg[desk] = rows
strong = {d: [r for r in v if r and r[0] >= .35] for d, v in per_leg.items()}
for d, v in strong.items():
    if len(v) >= 2:
        sh = np.mean([r[1] for r in v], axis=0)
        print("%-15s legs with evidence>=0.35: %d  mean offset (%.2f, %.2f) m  peaks %s" % (d, len(v), sh[0], sh[1], [round(r[0], 2) for r in v]))
    else:
        print("%-15s legs with evidence>=0.35: %d  (observed in maps: %d legs)" % (d, len(v), sum(1 for r in per_leg[d] if r)))
json.dump(dict(slide=list(map(float, slide)), per_leg=per_leg), open(OUT + "/deskfit_legs.json", "w"))
