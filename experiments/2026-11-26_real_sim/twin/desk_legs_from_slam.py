"""Register physical SLAM maps (last_maps.py output) to one room frame; keep recurring small blobs.

usage: desk_legs_from_slam.py <maps dir> <out dir> <map1,map2,...>  (first map = reference)
Writes legs.json/legs.png (blob clusters), legs2.json/legs2.png and evidence.npz (occupancy share).
"""
import glob, json, math, os, sys
import numpy as np, cv2
from scipy import ndimage
from scipy.spatial import cKDTree
D, OUT = sys.argv[1], sys.argv[2]
names = sys.argv[3].split(",")

def load(n):
    z = np.load(f"{D}/{n}.npz"); return z["data"], float(z["resolution"]), z["origin"]

def room_frame(d, res, o):
    free = ((d >= 0) & (d <= 25)).astype(np.uint8)
    free = cv2.morphologyEx(free, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    cs, _ = cv2.findContours(free, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    c = max(cs, key=cv2.contourArea)
    (cx, cy), (w, h), ang = cv2.minAreaRect(c)
    if w < h: ang += 90; w, h = h, w
    centre = np.array([o[0] + (cx + .5) * res, o[1] + (cy + .5) * res])
    return centre, math.radians(ang), (w * res, h * res)

def to_room(pts, centre, ang):
    c, s = math.cos(-ang), math.sin(-ang); p = pts - centre
    return np.stack((c * p[:, 0] - s * p[:, 1], s * p[:, 0] + c * p[:, 1]), axis=1)

def occupied_points(d, res, o):
    rr, cc = np.nonzero(d >= 65)
    return np.stack((o[0] + (cc + .5) * res, o[1] + (rr + .5) * res), axis=1)

def small_blobs(d, res, o, max_extent=.25):
    lab, n = ndimage.label(d >= 65, structure=np.ones((3, 3)))
    out = []
    for i, sl in enumerate(ndimage.find_objects(lab), 1):
        ext = max(sl[0].stop - sl[0].start, sl[1].stop - sl[1].start) * res
        if ext <= max_extent:
            rr, cc = np.nonzero(lab[sl] == i)
            out.append((o[0] + (cc.mean() + sl[1].start + .5) * res, o[1] + (rr.mean() + sl[0].start + .5) * res))
    return np.array(out)

def big_points(d, res, o, min_extent=1.0):
    lab, n = ndimage.label(d >= 65, structure=np.ones((3, 3)))
    keep = [i + 1 for i, sl in enumerate(ndimage.find_objects(lab))
            if sl and max(sl[0].stop - sl[0].start, sl[1].stop - sl[1].start) * res >= min_extent]
    rr, cc = np.nonzero(np.isin(lab, keep))
    return np.stack((o[0] + (cc + .5) * res, o[1] + (rr + .5) * res), axis=1)

def icp(src, tree, iters=40, gate=(.6, .3, .15)):
    R, t = np.eye(2), np.zeros(2)
    for k in range(iters):
        q = src @ R.T + t
        g = gate[min(k // 12, len(gate) - 1)]
        dist, idx = tree.query(q, distance_upper_bound=g)
        ok = np.isfinite(dist)
        if ok.sum() < 50: break
        a, b = q[ok], tree.data[idx[ok]]
        ma, mb = a.mean(0), b.mean(0)
        U, _, Vt = np.linalg.svd((a - ma).T @ (b - mb))
        Ri = (U @ Vt).T
        if np.linalg.det(Ri) < 0: Vt[-1] *= -1; Ri = (U @ Vt).T
        R, t = Ri @ R, Ri @ t + (mb - Ri @ ma)
    dist, _ = tree.query(src @ R.T + t, distance_upper_bound=.3)
    inlier = np.isfinite(dist)
    return R, t, float(np.mean(dist[inlier])) if inlier.any() else 9., float(inlier.mean())

ref = None; frames = {}; report = []
for n in names:
    d, res, o = load(n)
    centre, ang, size = room_frame(d, res, o)
    walls = big_points(d, res, o)
    best = None
    for flip in (0., math.pi):
        base = to_room(walls, centre, ang + flip)
        if ref is None:
            best = (0., 1., flip, np.eye(2), np.zeros(2)); break
        R, t, err, frac = icp(base[::2], ref)
        if best is None or (frac, -err) > (best[1], -best[0]):
            best = (err, frac, flip, R, t)
    err, frac, flip, R, t = best
    frames[n] = (centre, ang + flip, R, t)
    aligned = to_room(walls, centre, ang + flip) @ R.T + t
    if ref is None:
        ref = cKDTree(aligned)
    report.append((n, round(size[0], 2), round(size[1], 2), round(err, 3), round(frac, 2)))
for r in report: print("map", r[0], "room %.2f x %.2f m" % (r[1], r[2]), "inlier mean dist %.3f m, inlier share %.2f" % (r[3], r[4]))
# small blobs in the shared room frame
allb = []
for k, n in enumerate(names):
    d, res, o = load(n); centre, ang, R, t = frames[n]
    b = small_blobs(d, res, o)
    if len(b): allb += [(*p, k) for p in to_room(b, centre, ang) @ R.T + t]
allb = np.array(allb)
tree = cKDTree(allb[:, :2]); used = np.zeros(len(allb), bool); clusters = []
for i in np.argsort(allb[:, 0]):
    if used[i]: continue
    idx = [j for j in tree.query_ball_point(allb[i, :2], .12) if not used[j]]
    used[idx] = True
    maps = len(set(allb[idx, 2].astype(int)))
    clusters.append((*allb[idx, :2].mean(axis=0), maps))
clusters = np.array(clusters)
need = max(3, len(names) // 2)
legs = clusters[clusters[:, 2] >= need]
print("blobs", len(allb), "clusters", len(clusters), f"recurring in >= {need} of {len(names)} maps:", len(legs))
json.dump(dict(maps=names, frames={n: [list(map(float, f[0])), float(f[1]), f[2].tolist(), list(map(float, f[3]))] for n, f in frames.items()},
               legs=legs.tolist(), all_clusters=clusters.tolist()), open(f"{OUT}/legs.json", "w"))
# picture: reference walls + legs
res = .05; W, H = 26., 18.
img = np.full((int(H / res), int(W / res), 3), 255, np.uint8)
def px(p): return int((p[0] + W / 2) / res), int((H / 2 - p[1]) / res)
for n in names:
    d, r_, o = load(n); centre, ang, R, t = frames[n]
    for p in (to_room(occupied_points(d, r_, o), centre, ang) @ R.T + t)[::3]:
        x, y = px(p)
        if 0 <= x < img.shape[1] and 0 <= y < img.shape[0]: img[y, x] = (190, 190, 190)
for x, y, m in clusters:
    cv2.circle(img, px((x, y)), 2, (0, 0, 255) if m >= need else (255, 160, 160), -1)
cv2.imwrite(f"{OUT}/legs.png", cv2.resize(img, None, fx=1.6, fy=1.6))

# --- evidence grid: share of maps (that observed a cell) in which it is occupied
res = .05; W, H = 26., 18.
shape = (int(H / res), int(W / res))
occ_n = np.zeros(shape, np.int32); known_n = np.zeros(shape, np.int32)
def cells(pts):
    c = ((pts[:, 0] + W / 2) / res).astype(int); r = ((pts[:, 1] + H / 2) / res).astype(int)
    ok = (c >= 0) & (c < shape[1]) & (r >= 0) & (r < shape[0]); return r[ok], c[ok]
for n in names:
    d, r_, o = load(n); centre, ang, R, t = frames[n]
    rr, cc = np.nonzero(d >= 0)
    kp = to_room(np.stack((o[0] + (cc + .5) * r_, o[1] + (rr + .5) * r_), 1), centre, ang) @ R.T + t
    k = np.zeros(shape, bool); k[cells(kp)] = True
    oc = np.zeros(shape, bool); oc[cells(to_room(occupied_points(d, r_, o), centre, ang) @ R.T + t)] = True
    oc = ndimage.binary_dilation(oc, iterations=1)
    occ_n += oc; known_n += ndimage.binary_dilation(k, iterations=1)
share = np.where(known_n >= 3, occ_n / np.maximum(known_n, 1), 0.)
stable = share >= .5
lab, n = ndimage.label(stable, structure=np.ones((3, 3)))
legs2 = []
for i, sl in enumerate(ndimage.find_objects(lab), 1):
    ext = max(sl[0].stop - sl[0].start, sl[1].stop - sl[1].start) * res
    if ext > .35: continue
    rr, cc = np.nonzero(lab[sl] == i)
    x = (cc.mean() + sl[1].start + .5) * res - W / 2; y = (rr.mean() + sl[0].start + .5) * res - H / 2
    legs2.append((x, y, float(share[sl][lab[sl] == i].max()), int(known_n[sl][lab[sl] == i].max())))
legs2 = np.array(legs2)
print("stable small blobs:", len(legs2))
img = np.full(shape + (3,), 255, np.uint8)
img[share > 0] = (235, 235, 235)
img[(share >= .5) & ~np.isin(lab, [i + 1 for i in range(len(legs2))])] = (150, 150, 150)
img[stable] = (150, 150, 150)
for x, y, sh, kn in legs2:
    cv2.circle(img, (int((x + W / 2) / res), int((y + H / 2) / res)), 3, (0, 0, 255), -1)
cv2.imwrite(f"{OUT}/legs2.png", cv2.resize(cv2.flip(img, 0), None, fx=1.6, fy=1.6, interpolation=cv2.INTER_NEAREST))
json.dump(dict(maps=names, legs=legs2.tolist(), room_frame_note="x along the room long axis, origin at the room centre of the reference map"), open(f"{OUT}/legs2.json", "w"))
np.savez_compressed(f"{OUT}/evidence.npz", share=share, known=known_n, res=res, W=W, H=H)
