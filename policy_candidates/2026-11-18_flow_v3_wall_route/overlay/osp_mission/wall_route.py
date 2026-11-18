"""Side-surface inspection and path-level revisit costs in the odometry frame."""
from collections import deque
import math

import numpy as np
from rclpy.time import Time


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def camera_pose(transform):
    p, q = transform.translation, transform.rotation
    dx, dy = 2*(q.x*q.z+q.w*q.y), 2*(q.y*q.z-q.w*q.x)
    if math.hypot(dx, dy) < .65:
        return None
    return p.x, p.y, math.atan2(dy,dx)


class WallLook:
    def __init__(self):
        self.next_check = 0.
        self.recent = deque(maxlen=64)
        self.pending = None

    def select(self, scan, laser, camera, fov, now):
        ranges = np.asarray(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges))*scan.angle_increment + laser[2]
        good = np.isfinite(ranges) & (ranges>scan.range_min) & (ranges<scan.range_max)
        indices = np.flatnonzero(good)
        xy = np.column_stack((laser[0]+ranges[good]*np.cos(angles[good]),
                              laser[1]+ranges[good]*np.sin(angles[good])))
        # Never fit across a missing return, a corner-sized jump, or the scan seam.
        cuts = np.flatnonzero((np.diff(indices)>1) | (np.linalg.norm(np.diff(xy,axis=0),axis=1)>.14))+1
        best = None
        def consider(center):
            nonlocal best
            delta = center-camera[:2]
            dist = np.linalg.norm(delta)
            if not .42<=dist<=1.65:
                return
            direction = math.atan2(delta[1],delta[0])
            turn = wrap(direction-camera[2])
            if not max(fov*.5,.42)<abs(turn)<=1.85:
                return
            if any(now-t<100. and math.dist(center,old)<1.0 and
                   abs(wrap(direction-old_direction))<math.radians(55)
                   for t,old,old_direction in self.recent):
                return
            score = abs(dist-.85)+.18*abs(turn)
            if best is None or score<best[0]:
                best=(score,tuple(center))

        for segment in np.split(xy,cuts):
            if len(segment)<6:
                continue
            arc = np.r_[0.,np.cumsum(np.linalg.norm(np.diff(segment,axis=0),axis=1))]
            # A short coherent object silhouette can carry a marker even on a curved face.
            width=np.linalg.norm(segment[-1]-segment[0])
            if .12<=width<=.70 and arc[-1]<=1.7*width:
                consider(segment[len(segment)//2])
            for start in np.arange(0.,max(0.,arc[-1]-.35),.25):
                i,j = np.searchsorted(arc,[start,start+.40])
                points = segment[i:j]
                if len(points)<6 or np.linalg.norm(points[-1]-points[0])<.30:
                    continue
                center = points.mean(axis=0)
                delta = center-camera[:2]
                edge = points[-1]-points[0]
                normal = np.array([-edge[1],edge[0]])/np.linalg.norm(edge)
                if np.max(np.abs((points-center)@normal))>.035 or abs(delta@normal)<.75*np.linalg.norm(delta):
                    continue
                consider(center)
        return best[1] if best else None

    def check(self, node, now):
        if self.pending:
            if now-self.pending[0]<=3.:
                return True
            self.pending=None
        if now<self.next_check or now-node.contact_inspection.last_look<12.:
            return False
        self.next_check=now+1.
        scan=node.latest_scan
        calibration=getattr(node,'inspection_camera',None)
        if scan is None or not calibration or not node.sensor_fresh(node.scan_stamp):
            return False
        try:
            stamp=Time.from_msg(scan.header.stamp)
            def pose(frame):
                try:
                    return node.tf.lookup_transform('odom',frame,stamp).transform
                except Exception:
                    latest=node.tf.lookup_transform('odom',frame,Time())
                    if abs((Time.from_msg(latest.header.stamp)-stamp).nanoseconds)>200_000_000:
                        raise
                    return latest.transform
            laser=pose(scan.header.frame_id)
            cam=camera_pose(pose(node.inspection_camera_frame))
            if cam is None:
                return False
            point=self.select(scan,(laser.translation.x,laser.translation.y,yaw(laser.rotation)),cam,calibration[1],now)
        except Exception as exc:
            node.get_logger().debug(f'Wall observation unavailable: {exc}')
            return False
        if point is None:
            return False
        self.pending=(now,point)
        return True

    def consume(self,node,now):
        request,self.pending=self.pending,None
        if request is None or now-request[0]>3.:
            return None
        try:
            cam=camera_pose(node.tf.lookup_transform('odom',node.inspection_camera_frame,Time()).transform)
        except Exception:
            return None
        if cam is None:
            return None
        x,y=request[1]
        turn=wrap(math.atan2(y-cam[1],x-cam[0])-cam[2])
        if abs(turn)>1.95 or math.hypot(y-cam[1],x-cam[0])>1.8:
            return None
        self.recent.append((now,(x,y),math.atan2(y-cam[1],x-cam[0])))
        node.get_logger().info(f'Facing potential marker surface: {math.degrees(turn):.0f} deg')
        return turn


def penalize_routes(goals, previous, shape, resolution, origin, transform, cells):
    """Average traversal history over the first 3 m, excluding the common start cell."""
    if previous is None or not cells or len(goals)<2:
        return
    tx,ty,angle=transform
    c,s=math.cos(angle),math.sin(angle)
    width=shape[1]
    cache={}
    def location(index):
        row,col=divmod(index,width)
        x,y=origin[0]+(col+.5)*resolution,origin[1]+(row+.5)*resolution
        return math.floor((tx+c*x-s*y)/.6),math.floor((ty+s*x+c*y)/.6)
    scores=[]
    for goal in goals:
        if 'cell' not in goal:
            scores.append(0.)
            continue
        row,col=goal['cell']
        index=int(row*width+col)
        trail=[]
        while index>=0 and index not in cache:
            trail.append(index)
            index=int(previous[index])
        for idx in reversed(trail):
            parent=int(previous[idx])
            if parent<0:
                cache[idx]=(0.,0.,location(idx))
                continue
            length,total,first=cache[parent]
            step=min(max(0.,3.-length),resolution*math.hypot(idx//width-parent//width,idx%width-parent%width))
            cell=location(idx)
            count=cells.get(cell,{}).get('entries',0) if cell!=first else 0
            cache[idx]=(length+step,total+step*min(3,count),first)
        length,total,_=cache[int(row*width+col)]
        scores.append(total/max(.6,length))
    baseline=min(scores)
    for goal,score in zip(goals,scores):
        goal['features'][4]+=6.*(score-baseline)
