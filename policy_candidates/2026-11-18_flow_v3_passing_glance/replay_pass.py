"""Compare two re-observation policies on synchronized recorded frames, without motion."""
import argparse
import json
import tempfile
from pathlib import Path
from time import perf_counter_ns

from passing import EdgeGlance, PassingGlance
from replay_glance import (SquarePriority, borders, can_confirm, cv2, decode, far_face,
                           local_observations, mission, np, plane_hit, preview_geometry, replay)


def surface_request(state, candidates, scan, origin, camera, cal, image, stamp, now):
    samples = list(local_observations(scan,origin,camera,cal,image)) if candidates else []
    tracks, pixels, confirmable = [], [], set()
    for pixel in candidates:
        u, _, size = pixel
        hit = plane_hit(u,size,samples,camera,cal)
        if hit is None:
            hit = far_face(u,size,scan,origin,camera,cal)
        if hit is None:
            continue
        if can_confirm(u,size,image.shape[1]):
            confirmable.add(len(tracks))
        tracks.append((*hit,size))
        pixels.append(pixel)
    n = len(state.looks)
    state.track_frame(tracks,stamp,now,camera,confirmable=confirmable)
    if len(state.looks)==n:
        return None
    event = state.looks[-1]
    i = min(range(len(tracks)),key=lambda j: np.linalg.norm(tracks[j][0]-event['point']))
    u,v,size = pixels[i]
    return dict(action=event['action'],source='confirmed',uv=[u,v],size=size,
                yaw_delta_deg=event['yaw_delta_deg'],dwell_s=event['dwell_s'])


def score(run, output):
    old, new, state = EdgeGlance(), PassingGlance(), SquarePriority()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    start = mission(run)
    events, reads, timings = [], {}, []
    folder = output/run.name
    folder.mkdir(parents=True,exist_ok=True)
    def frame(now, stamp, scan, origin, camera, info, image):
        begin = perf_counter_ns()
        cal = preview_geometry(info)
        if cal is info:
            raise ValueError('Unsupported camera calibration')
        candidates = borders(image)
        confirmed = surface_request(state,candidates,scan,origin,camera,cal,image,stamp,now)
        shared = perf_counter_ns()
        a = old.update(now,stamp,camera,image.shape[1],cal.k[0],cal.k[2],candidates,confirmed)
        old_end = perf_counter_ns()
        b = new.update(now,stamp,camera,image,cal.k[0],cal.k[2],candidates,confirmed)
        new_end = perf_counter_ns()
        quads, ids = decode(image,dictionary)
        end = perf_counter_ns()
        timings.append([(shared-begin)/1e6,(old_end-shared)/1e6,(new_end-old_end)/1e6,(end-new_end)/1e6])
        t = round(now-start,3)
        for quad, mid in zip(quads,[] if ids is None else ids.ravel()):
            if np.linalg.norm(quad[0]-np.roll(quad[0],1,axis=0),axis=1).min()>=18:
                reads.setdefault(int(mid),t)
        if a or b:
            events.append(dict(t=t,old=a,new=b))
            cv2.imwrite(str(folder/f'{t:.3f}.jpg'),image)
    with tempfile.TemporaryDirectory() as tmp:
        local = Path(tmp)/run.name
        (local/'artifacts').mkdir(parents=True)
        for path in ('rosbag','artifacts/startup.jsonl'):
            (local/path).symlink_to(run.resolve()/path)
        replay(local,Path(tmp),inspect_frame=frame)
    measured = np.asarray(timings)[10:]  # omit startup frames from per-run latency
    np.save(folder/'timing.npy', measured)
    result = dict(run=run.name,frames=len(timings),reads=reads,events=events)
    (folder/'summary.json').write_text(json.dumps(result))
    print(run.name,len(timings),len(events),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path)
    parser.add_argument('out',type=Path)
    parser.add_argument('cohort',choices=['development','reviewed','holdout'])
    args=parser.parse_args()
    cv2.setNumThreads(1)
    cohorts=json.loads(Path(__file__).with_name('cohorts.json').read_text())
    for name in cohorts[args.cohort]:
        score(args.root/name,args.out)
