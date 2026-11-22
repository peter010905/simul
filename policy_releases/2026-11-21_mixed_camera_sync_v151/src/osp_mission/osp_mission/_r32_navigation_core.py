"""ROS-independent geometry, target filtering, and exploration algorithms."""
from collections import deque
from dataclasses import dataclass, field
import math
import copy

import numpy as np
from scipy import ndimage


# Shortest marker edge, in pixels, that perception accepts for a metric fix.
MARKER_MIN_EDGE_PX = 14


def marker_detection_range(fx, marker_size, min_edge_px=MARKER_MIN_EDGE_PX):
    """Frontal distance at which a marker of marker_size metres spans min_edge_px."""
    return fx*marker_size/min_edge_px


def marker_edge_metres(polygon, depth, fx):
    """Physical length of the longest image edge of a quad seen at depth metres.

    Foreshortening only shrinks an edge, so the longest one is a lower bound of
    the real side: a box face decoded as a marker still measures as a box face.
    """
    polygon = np.asarray(polygon,dtype=float).reshape(4,2)
    return float(np.max(np.linalg.norm(polygon-np.roll(polygon,1,axis=0),axis=1))*depth/fx)


def rotation(q):
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def transform(points, translation, quaternion):
    return np.asarray(points) @ rotation(quaternion).T + np.asarray(translation)


def depth_point(depth, corners, camera_matrix, scale, min_depth, max_depth):
    """Median valid depth inside an inset polygon; depth must be RGB-aligned."""
    import cv2
    center = np.mean(corners, axis=0)
    polygon = np.rint(center + 0.55*(corners-center)).astype(np.int32)
    mask = np.zeros(depth.shape, np.uint8)
    cv2.fillConvexPoly(mask, polygon, 1)
    values = depth[mask.astype(bool)].astype(float)*scale
    valid = values[np.isfinite(values) & (values >= min_depth) & (values <= max_depth)]
    if len(valid) < 8 or len(valid) < 0.4*len(values):
        return None
    z = float(np.median(valid))
    if float(np.median(np.abs(valid-z))) > 0.08*z:
        return None
    ray = np.linalg.solve(camera_matrix, [center[0], center[1], 1.0])
    return ray*(z/ray[2])


def lidar_intersection(ray_origin, ray_direction, scan_points, max_angle=0.10):
    """Intersect camera bearing with a local vertical surface from 2D laser points.

    All input coordinates are in base_link, whose z axis must be vertical.
    A local line is fitted only to a contiguous scan cluster in the bearing cone.
    """
    points = np.asarray(scan_points)
    if len(points) < 4:
        return None
    direction = np.asarray(ray_direction)[:2]
    norm = np.linalg.norm(direction)
    if norm < 1e-6:
        return None
    direction = direction/norm
    origin = np.asarray(ray_origin)[:2]
    delta = points[:, :2]-origin
    along = delta @ direction
    across = delta[:, 0]*direction[1]-delta[:, 1]*direction[0]
    selected = np.flatnonzero((along > 0.15) &
                             (np.abs(np.arctan2(across, along)) < max_angle))
    if len(selected) < 4:
        return None
    # Do not fit across separate objects or a missing-scan gap.
    breaks = np.flatnonzero((np.diff(selected) > 1) |
                           (np.linalg.norm(np.diff(points[selected, :2], axis=0), axis=1) > .15))+1
    groups = [g for g in np.split(selected, breaks) if len(g) >= 4]
    candidates = []
    for group in groups:
        p = points[group, :2]
        center = p.mean(axis=0)
        _, _, vt = np.linalg.svd(p-center, full_matrices=False)
        tangent, normal = vt
        if np.sqrt(np.mean(((p-center) @ normal)**2)) > .035:
            continue
        denominator = float(direction @ normal)
        if abs(denominator) < .2:
            continue
        distance = float((center-origin) @ normal)/denominator
        hit = origin + distance*direction
        extent = (p-center) @ tangent
        offset = float((hit-center) @ tangent)
        if distance <= 0 or offset < extent.min()-.04 or offset > extent.max()+.04:
            continue
        candidates.append((distance, hit))
    if not candidates:
        return None
    _, hit = min(candidates, key=lambda item: item[0])
    return np.array([hit[0], hit[1], 0.0])


@dataclass
class Grid:
    cells: np.ndarray
    resolution: float
    origin: tuple

    def world(self, row, col):
        x, y = (col+.5)*self.resolution, (row+.5)*self.resolution
        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([ox+c*x-s*y, oy+s*x+c*y])

    def index(self, x, y):
        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = x-ox, y-oy
        return math.floor((-s*dx+c*dy)/self.resolution), math.floor((c*dx+s*dy)/self.resolution)

    def inside(self, row, col):
        return 0 <= row < self.cells.shape[0] and 0 <= col < self.cells.shape[1]

    def observed_free(self,polygon):
        """Fill unknown cells inside a recent LiDAR free-space polygon, preserving obstacles."""
        import cv2
        points = np.int32([self.index(*p)[::-1] for p in polygon])
        inside = np.zeros(self.cells.shape,dtype=np.uint8)
        cv2.fillPoly(inside,[points],1)
        cells = self.cells.copy()
        cells[inside.astype(bool) & (cells < 0)] = 0
        return Grid(cells,self.resolution,self.origin)

    def reachable(self, robot, clearance, minimum_clearance=.23):
        free = (self.cells >= 0) & (self.cells <= 25)
        # Padding makes the map edge an obstacle as well as occupied/unknown cells.
        distance = ndimage.distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1]
        safe = free & (distance*self.resolution >= clearance)
        row, col = self.index(*robot)
        if not self.inside(row, col):
            return safe, np.full(safe.shape, np.inf)
        traversable = safe.copy()
        if not safe[row,col]:
            # Connect only this start to the normal safe region. The rest of the
            # map retains 0.30m clearance; never globally shrink obstacles.
            hard = free & (distance*self.resolution >= min(clearance,minimum_clearance))
            if not hard[row,col]:
                return safe,np.full(safe.shape,np.inf)
            queue = deque([(row,col)])
            previous = {(row,col):None}
            endpoint = None
            while queue:
                r,c = queue.popleft()
                if safe[r,c]:
                    endpoint = (r,c)
                    break
                for nr,nc in ((r-1,c),(r+1,c),(r,c-1),(r,c+1)):
                    if not self.inside(nr,nc) or (nr,nc) in previous or not hard[nr,nc]:
                        continue
                    if math.hypot(nr-row,nc-col)*self.resolution > 1.0:
                        continue
                    # Recovery must not move closer to an obstacle/unknown area.
                    if distance[nr,nc]+1e-8 < distance[r,c]:
                        continue
                    previous[nr,nc] = (r,c)
                    queue.append((nr,nc))
            if endpoint is None:
                return safe,np.full(safe.shape,np.inf)
            while endpoint is not None:
                traversable[endpoint] = True
                endpoint = previous[endpoint]
        distances = np.full(safe.shape, np.inf)
        distances[row, col] = 0
        queue = deque([(row, col)])
        # Four-connected paths avoid diagonal corner cutting.
        while queue:
            r, c = queue.popleft()
            for nr, nc in ((r-1,c), (r+1,c), (r,c-1), (r,c+1)):
                if self.inside(nr, nc) and traversable[nr,nc] and np.isinf(distances[nr,nc]):
                    distances[nr,nc] = distances[r,c]+self.resolution
                    queue.append((nr,nc))
        return safe, distances

    def line_clear(self, start, target, allow_unknown=False):
        a, b = np.array(start), np.array(target)
        n = max(2, int(np.linalg.norm(b-a)/self.resolution)*2)
        # Last cell may be the wall carrying the marker.
        for p in np.linspace(a,b,n)[:-2]:
            r,c = self.index(*p)
            if not self.inside(r,c) or (self.cells[r,c] < 0 and not allow_unknown) or self.cells[r,c] > 25:
                return False
        return True


def choose_goal(grid, robot, clearance, visited=(), blocked=(), target=None,reachability=None,seen_from=None,view_blocked=()):
    """Returns x,y,yaw,kind; fallback coverage samples previously unseen viewpoints."""
    safe, distances = grid.reachable(robot, clearance) if reachability is None else reachability
    reachable = safe & np.isfinite(distances)
    if not reachable.any():
        return None
    rows, cols = np.where(reachable)
    # Thin to ~25 cm candidates to bound scoring cost.
    stride = max(1, round(.25/grid.resolution))
    keep = (rows % stride == 0) & (cols % stride == 0)
    rows, cols = rows[keep], cols[keep]
    if not len(rows):
        return None
    points = np.array([grid.world(r,c) for r,c in zip(rows,cols)])
    valid = distances[rows,cols] >= .35
    for p in blocked:
        valid &= np.linalg.norm(points-np.asarray(p), axis=1) > .65
    if target is not None:
        delta = np.asarray(target)-points
        ranges = np.linalg.norm(delta, axis=1)
        valid &= (ranges >= .75) & (ranges <= 1.6)
        for point in view_blocked:
            valid &= np.linalg.norm(points-np.asarray(point),axis=1)>.30
        score = distances[rows,cols] + 2*np.abs(ranges-1.2)
        if seen_from is not None:
            visible_side = np.asarray(seen_from)[:2]-target
            alignment = ((points-np.asarray(target)) @ visible_side)/np.maximum(ranges*np.linalg.norm(visible_side),.001)
            valid &= alignment>.25
            # Keep the proven sightline and take the shortest useful approach.
            # A larger lateral move can put a thin table leg in front of the tag.
            score = distances[rows,cols]+2*np.abs(ranges-1.6)+4*(1-alignment)
            order = np.lexsort((score,alignment<.9))
        else:
            order = np.argsort(score)
        for i in order:
            # Only a measured marker gets endpoint tolerance: the occupied cell
            # carrying its paper is not an intervening obstacle. Keep the goal
            # itself collision-free, and remain on the side it was observed from.
            endpoint = (np.asarray(target)-.15*delta[i]/max(ranges[i],.001)
                        if seen_from is not None else target)
            if valid[i] and grid.line_clear(points[i],endpoint):
                return (*points[i], math.atan2(delta[i,1],delta[i,0]), 'approach')
        return None
    unknown = grid.cells < 0
    free = (grid.cells >= 0) & (grid.cells <= 25)
    frontier = free & ndimage.binary_dilation(unknown)
    labels, count = ndimage.label(frontier)
    best = None
    for label in range(1,count+1):
        fr, fc = np.where(labels == label)
        if len(fr) < 3:
            continue
        center = grid.world(float(np.mean(fr)),float(np.mean(fc)))
        ranges = np.linalg.norm(points-center,axis=1)
        candidates = valid & (ranges < 1.3)
        # Recently observed locations are de-prioritized, not permanently banned.
        penalty = np.zeros(len(points))
        for p in visited:
            penalty += 3*np.exp(-np.linalg.norm(points-np.asarray(p),axis=1)**2/.5)
        score = distances[rows,cols] + 2*ranges + penalty - .1*min(len(fr),30)
        score[~candidates] = np.inf
        i = int(np.argmin(score))
        if np.isfinite(score[i]) and (best is None or score[i] < best[0]):
            delta = center-points[i]
            best = (score[i], (*points[i],math.atan2(delta[1],delta[0]),'frontier'))
    if best:
        return best[1]
    # No frontiers: observe reachable space not yet covered by a full scan.
    for p in visited:
        valid &= np.linalg.norm(points-np.asarray(p),axis=1) > 1.0
    score = distances[rows,cols].copy()
    score[~valid] = np.inf
    i = int(np.argmin(score))
    if np.isfinite(score[i]):
        return (*points[i],0.0,'coverage')
    return None


def surface_samples(grid,box_faces=None,include_thin=False):
    """Mapped wall segments plus inferred faces of small obstacles, in map coordinates."""
    import cv2
    occupied = grid.cells >= 65
    # Long mapped walls give a stable orthogonal basis when a box is only an L.
    # Estimate this orientation from sensors; never use world/scene coordinates.
    lines = cv2.HoughLinesP(occupied.astype(np.uint8)*255,1,math.pi/180,
                           threshold=max(10,int(1/grid.resolution)),
                           minLineLength=1.5/grid.resolution,maxLineGap=.2/grid.resolution)
    room_angle = None
    if lines is not None:
        delta = lines[:,0,2:]-lines[:,0,:2]
        weights = np.linalg.norm(delta,axis=1)**2
        angles = np.arctan2(delta[:,1],delta[:,0])
        direction = np.sum(weights*np.exp(4j*angles))
        if abs(direction)/np.sum(weights) > .8:
            room_angle = float(np.angle(direction)/4)+grid.origin[2]
    samples = []
    # Sample long walls in overlapping camera-sized sections. Both normals are
    # considered; only the side with a reachable free observation pose survives.
    if lines is not None:
        for x1,y1,x2,y2 in lines[:,0]:
            a,b = grid.world(y1,x1),grid.world(y2,x2)
            delta = b-a
            length = np.linalg.norm(delta)
            normal = np.array([-delta[1],delta[0]])/length
            for fraction in (np.arange(max(1,math.ceil(length/.8)))+.5)/max(1,math.ceil(length/.8)):
                point = a+fraction*delta
                samples.extend([(point,normal),(point,-normal)])
    labels,count = ndimage.label(ndimage.binary_dilation(occupied),structure=np.ones((3,3)))
    for label in range(1,count+1):
        rows,cols = np.where((labels == label) & occupied)
        if len(rows) < (2 if include_thin else 5) or max(np.ptp(rows),np.ptp(cols))*grid.resolution > 3.5:
            continue
        points = np.float32([grid.world(r,c) for r,c in zip(rows,cols)])
        center,size,angle = cv2.minAreaRect(points)
        if room_angle is not None:
            basis = np.array([[math.cos(room_angle),-math.sin(room_angle)],
                              [math.sin(room_angle),math.cos(room_angle)]])
            projected = points @ basis
            lo,hi = projected.min(axis=0),projected.max(axis=0)
            center = basis @ ((lo+hi)/2)
            size = hi-lo
            angle = math.degrees(room_angle)
        if (not include_thin and max(size) < .3) or max(size) > 2.8:
            continue
        center = np.array(center)
        angle = math.radians(angle)
        u,v = np.array([math.cos(angle),math.sin(angle)]),np.array([-math.sin(angle),math.cos(angle)])
        half = np.maximum(size,grid.resolution if include_thin else .3)/2
        faces = [(center+normal*extent,normal)
                 for normal,extent in [(u,half[0]),(-u,half[0]),(v,half[1]),(-v,half[1])]]
        samples.extend(faces)
        if box_faces is not None:
            box_faces.append(faces)
    return deduplicate_surfaces(np.asarray([(*p,*n) for p,n in samples],dtype=float).reshape(-1,4))


def deduplicate_surfaces(samples):
    """Suppress overlapping Hough lines by physical distance and normal, not bins."""
    if not len(samples):
        return samples
    order = np.lexsort((samples[:,1],samples[:,0],np.arctan2(samples[:,3],samples[:,2])))
    selected = []
    for i in order:
        if selected:
            previous = samples[selected]
            duplicate = (np.linalg.norm(previous[:,:2]-samples[i,:2],axis=1)<.60) & (previous[:,2:]@samples[i,2:]>.94)
            if duplicate.any():
                continue
        selected.append(i)
    return samples[selected]


def visible_surfaces(grid,surfaces,pose,max_range=2.2):
    """Camera coverage proxy; actual ArUco detection remains a separate measurement."""
    delta = surfaces[:,:2]-np.asarray(pose[:2])
    distance = np.linalg.norm(delta,axis=1)
    front = np.sum(-delta*surfaces[:,2:],axis=1)/np.maximum(distance,.001)
    # Oblique surfaces occupy fewer image pixels: don't credit distant grazing views.
    # A close transit can crop a large marker vertically even when its supporting
    # surface is in front of the camera. Do not retire that face as inspected.
    valid = (distance > .7) & (distance < max_range*front) & (front > .45)
    if pose[2] is not None:
        angle = np.arctan2(delta[:,1],delta[:,0])-pose[2]
        valid &= np.abs(np.arctan2(np.sin(angle),np.cos(angle))) < .5
    for i in np.flatnonzero(valid):
        # End just outside the sampled surface to avoid rasterized wall thickness.
        valid[i] = grid.line_clear(pose[:2],surfaces[i,:2]+surfaces[i,2:]*.10,allow_unknown=True)
    return valid


def box_corner_candidates(grid,faces,reachability,blocked,max_range):
    """Use one camera heading for two adjacent faces of an inferred box."""
    safe,distances = reachability
    for a,na in faces[:2]:
        for b,nb in faces[2:]:
            pair = np.array([(*a,*na),(*b,*nb)])
            midpoint = (a+b)/2
            corner = a+nb*np.dot(b-a,nb)
            for offset in (.85,.65):
                view = corner+(na+nb)*offset
                r,c = grid.index(*view)
                if not grid.inside(r,c) or not safe[r,c] or not np.isfinite(distances[r,c]) or distances[r,c]<.25:
                    continue
                if any(np.linalg.norm(view-p)<.65 for p in blocked):
                    continue
                bearing = math.atan2(midpoint[1]-view[1],midpoint[0]-view[0])
                if visible_surfaces(grid,pair,(*view,bearing),max_range).all():
                    yield (*view,bearing,'corner')
                    break


def observed_surface_mask(surfaces,observations):
    """Match only surfaces present when camera evidence was collected."""
    covered = np.zeros(len(surfaces),dtype=bool)
    for old in observations:
        covered |= (np.linalg.norm(surfaces[:,:2]-old[:2],axis=1)<.25) & (surfaces[:,2:]@old[2:]>.94)
    return covered


def camera_surface_evidence(grid,poses,max_range,oblique_walls=False):
    """Require consecutive processed views and a clear patch around the face."""
    surfaces = surface_samples(grid)
    # Hough segment centers slide as SLAM fills a wall. Record its actually
    # visible occupied interval, not just one unstable representative center.
    min_cosine = .65
    if len(surfaces):
        # Only long walls receive oblique transit credit. Keep box faces strict.
        if oblique_walls:
            wall = surface_weights(grid,surfaces,2.,oriented=True) == 1.
            min_cosine = np.tile(np.where(wall,.45,.65),5)
        dense = []
        for offset in (-.4,-.2,0.,.2,.4):
            shifted = surfaces.copy()
            shifted[:,:2] += np.column_stack((-surfaces[:,3],surfaces[:,2]))*offset
            dense.append(shifted)
        surfaces = np.concatenate(dense)
        occupied_distance = ndimage.distance_transform_edt(grid.cells<65)*grid.resolution
        known = []
        for point in surfaces:
            r,c = grid.index(*(point[:2]-point[2:]*.05))
            known.append(grid.inside(r,c) and occupied_distance[r,c]<=.10)
        surfaces = surfaces[known]
        if oblique_walls:
            min_cosine = min_cosine[known]
    valid = np.ones(len(surfaces),dtype=bool)
    for pose in poses:
        valid &= visible_surface_patches(grid,surfaces,pose,max_range,min_cosine)
    return surfaces[valid]


def visible_surface_patches(grid,surfaces,pose,max_range,min_cosine=.65):
    valid = visible_surfaces(grid,surfaces,pose,max_range)
    delta = np.asarray(pose[:2])-surfaces[:,:2]
    valid &= np.sum(delta*surfaces[:,2:],axis=1)/np.maximum(np.linalg.norm(delta,axis=1),.001)>min_cosine
    for i in np.flatnonzero(valid):
        point,normal = surfaces[i,:2],surfaces[i,2:]
        tangent = np.array([-normal[1],normal[0]])
        valid[i] = all(grid.line_clear(pose[:2],point+normal*.10+tangent*offset,
                                      allow_unknown=False) for offset in (-.12,0.,.12))
    return valid


def inspection_candidates(grid,robot,clearance,inspected=(),blocked=(),reachability=None,
                          standoff=1.5,max_range=2.2,paired_box_views=False,box_groups=None,transit_views=(),include_thin=False,
                          observed_surfaces=None):
    safe,distances = grid.reachable(robot,clearance) if reachability is None else reachability
    row,col = grid.index(*robot[:2])
    can_turn_here = grid.inside(row,col) and safe[row,col]
    box_faces = [] if paired_box_views or box_groups is not None else None
    surfaces = surface_samples(grid,box_faces,include_thin)
    compact = np.zeros(len(surfaces),dtype=bool)
    if box_groups is not None:
        box_groups.extend(compact_box_groups(surfaces,box_faces,allow_partial=paired_box_views))
        for group in box_groups:
            compact |= group
    covered = np.zeros(len(surfaces),dtype=bool)
    if observed_surfaces is not None:
        covered = observed_surface_mask(surfaces,observed_surfaces)
    else:
        for pose in inspected:
            covered |= visible_surfaces(grid,surfaces,pose,max_range)
        for pose in transit_views:
            covered |= visible_surfaces(grid,surfaces,pose,.8*max_range)
    candidates = []
    visibility = visible_surfaces if observed_surfaces is None else visible_surface_patches
    available = np.zeros(len(surfaces),dtype=bool)
    for i,(x,y,nx,ny) in enumerate(surfaces):
        surface,normal = np.array([x,y]),np.array([nx,ny])
        stands = (1.2,.9,standoff) if compact[i] else (standoff,1.2,.9)
        offsets = (0.,-.4,.4) if observed_surfaces is not None else (0.,)
        for stand,offset in ((s,o) for s in stands for o in offsets):
            view = surface+normal*stand+np.array([-ny,nx])*offset
            r,c = grid.index(*view)
            if not grid.inside(r,c) or not safe[r,c] or not np.isfinite(distances[r,c]):
                continue
            if not grid.line_clear(view,surface+normal*.10,allow_unknown=True):
                continue
            heading = math.atan2(y-view[1],x-view[0])
            if not visibility(grid,surfaces[i:i+1],(*view,heading),max_range)[0]:
                continue
            available[i] = True
            # Grid path length can exceed Nav2's tolerance even when the actual
            # goal is already within it. Keep translation goals outside that
            # radius; the look candidate below observes from the real position.
            if covered[i] or np.linalg.norm(view-robot[:2]) < .4 or any(np.linalg.norm(view-p) < .65 for p in blocked):
                continue
            candidates.append((*view,heading,'inspect'))
            break
        # A new camera direction at the current safe position costs only a turn.
        bearing = math.atan2(y-robot[1],x-robot[0])
        look = (*robot[:2],bearing,'look')
        # A predicted view may yield no camera evidence. Do not repeat the same
        # settled view indefinitely, but keep the face unobserved and allow a
        # retry after other viewpoints instead of permanently retiring it.
        repeated_look = any(p[2] is not None and np.linalg.norm(np.asarray(robot[:2])-p[:2])<.25 and
            abs(math.atan2(math.sin(bearing-p[2]),math.cos(bearing-p[2])))<.2 for p in inspected[-8:])
        if can_turn_here and not covered[i] and not repeated_look and not any(np.linalg.norm(np.asarray(robot[:2])-p)<.4 for p in blocked):
            if visibility(grid,surfaces[i:i+1],look[:3],max_range)[0]:
                available[i] = True
                candidates.append(look)
    if paired_box_views:
        for faces in box_faces:
            for goal in box_corner_candidates(grid,faces,(safe,distances),blocked,max_range):
                visible = visibility(grid,surfaces,goal[:3],max_range)
                available |= visible
                if np.any(visible & ~covered):
                    candidates.append(goal)
    return candidates,surfaces,covered,available


def nav_traversable(costmap):
    # Nav2's OccupancyGrid encoding uses 99 for inscribed and 100 for lethal.
    return (costmap.cells >= 0) & (costmap.cells < 99)


def connected_nav_cells(costmap, robot):
    start = costmap.index(*robot[:2])
    traversable = nav_traversable(costmap)
    if not costmap.inside(*start) or not traversable[start]:
        return np.zeros_like(traversable)
    components, _ = ndimage.label(traversable)
    return components == components[start]


def nav_goal_reachability(grid, costmap, robot, reachability):
    """Keep SLAM goal cells that connect to the robot in Nav2's costmap."""
    safe, distances = reachability
    permitted = np.zeros_like(safe)
    connected = connected_nav_cells(costmap, robot)
    for row, col in zip(*np.nonzero(safe & np.isfinite(distances))):
        cell = costmap.index(*grid.world(row, col))
        if costmap.inside(*cell) and connected[cell]:
            permitted[row, col] = True
    return permitted, distances


def detour_waypoint(grid, costmap, robot, clearance, visited, blocked, blocked_radius,
                    side='either', reference_yaw=None):
    """Pick a short, reachable staging point outside the failed goal area."""
    start = costmap.index(*robot[:2])
    slam_start = grid.index(*robot[:2])
    if (not costmap.inside(*start) or not grid.inside(*slam_start)
            or not 0 <= grid.cells[slam_start] <= 25):
        return None
    traversable = nav_traversable(costmap)
    if not traversable[start]:
        return None
    # Use Nav2's footprint exclusion for the short departure, then require
    # the normal clearance from occupied and unknown cells at the waypoint.
    clear = ndimage.distance_transform_edt(np.pad(
        (costmap.cells >= 0) & (costmap.cells < 100), 1))[1:-1, 1:-1] * costmap.resolution
    max_steps = int(1.6 / costmap.resolution)
    queue = deque([(start, 0)])
    seen = {start}
    best, best_rank = None, None
    reference_yaw = robot[2] if reference_yaw is None else reference_yaw
    while queue:
        (row, col), steps = queue.popleft()
        if steps < max_steps:
            for cell in ((row-1,col),(row+1,col),(row,col-1),(row,col+1)):
                if cell in seen or not costmap.inside(*cell) or not traversable[cell]:
                    continue
                seen.add(cell)
                slam_cell = grid.index(*costmap.world(*cell))
                if not grid.inside(*slam_cell) or grid.cells[slam_cell] > 25:
                    continue
                queue.append((cell, steps + 1))
        if clear[row, col] < clearance:
            continue
        point = costmap.world(row, col)
        dx, dy = point - robot[:2]
        distance = math.hypot(dx, dy)
        if not .25 <= distance <= .85:
            continue
        r, c = grid.index(*point)
        if not grid.inside(r, c) or not 0 <= grid.cells[r, c] <= 25:
            continue
        if any(np.linalg.norm(point - previous) < .25 for previous in visited):
            continue
        if any(np.linalg.norm(point - failed) <= blocked_radius for failed in blocked):
            continue
        # Favor room to continue exploring, with a mild preference for lateral
        # moves and for cells with lower inflation cost in Nav2.
        heading = math.atan2(dy, dx)
        lateral = math.sin(heading - reference_yaw)
        preferred = ((side == 'right' and lateral * distance <= -.15)
                     or (side == 'left' and lateral * distance >= .15))
        forward = math.cos(heading - reference_yaw)
        score = (distance + .15 * abs(lateral) + .1 * forward
                 - .004 * costmap.cells[row, col] - .15 * steps * costmap.resolution)
        rank = (preferred, score)
        if best_rank is None or rank > best_rank:
            best, best_rank = (*point, heading, 'detour'), rank
    return best


def frontier_candidates(grid,robot,reachability,blocked,blocked_radius=.65):
    safe,distances = reachability
    free = (grid.cells >= 0) & (grid.cells <= 25)
    frontier = free & ndimage.binary_dilation(grid.cells < 0)
    rows,cols = np.where(frontier)
    if not len(rows):
        return []
    # Spatial bins keep long frontier boundaries from collapsing to one centroid.
    stride = max(1,round(1.5/grid.resolution))
    bins = np.column_stack((rows//stride,cols//stride))
    _,groups = np.unique(bins,axis=0,return_inverse=True)
    rr,cc = np.where(safe & np.isfinite(distances))
    thin = max(1,round(.3/grid.resolution))
    keep = (rr % thin == 0) & (cc % thin == 0)
    rr,cc = rr[keep],cc[keep]
    if not len(rr):
        return []
    points = np.array([grid.world(r,c) for r,c in zip(rr,cc)])
    valid = distances[rr,cc] >= .4
    for point in blocked:
        valid &= np.linalg.norm(points-point,axis=1) > blocked_radius
    goals = []
    for label in range(int(groups.max())+1):
        group = groups == label
        if group.sum()*grid.resolution < .20:
            continue
        center = grid.world(rows[group].mean(),cols[group].mean())
        ranges = np.linalg.norm(points-center,axis=1)
        scores = distances[rr,cc]*.2+ranges
        scores[~valid | (ranges > 1.3)] = np.inf
        k = int(np.argmin(scores))
        if np.isfinite(scores[k]):
            delta = center-points[k]
            goals.append((*points[k],math.atan2(delta[1],delta[0]),'frontier'))
    return goals


def mapping_gain(grid,position,occluders):
    """Expected unknown area from 360-degree range rays; never uses scene truth."""
    row,col = grid.index(*position)
    angles = np.linspace(0,2*math.pi,72,endpoint=False)-grid.origin[2]
    ranges = np.arange(.15,5.01,.10)/grid.resolution
    rr = np.floor(row+.5+np.sin(angles[:,None])*ranges).astype(int)
    cc = np.floor(col+.5+np.cos(angles[:,None])*ranges).astype(int)
    inside = (rr >= 0) & (cc >= 0) & (rr < grid.cells.shape[0]) & (cc < grid.cells.shape[1])
    values = np.full(rr.shape,-1,dtype=int)
    values[inside] = grid.cells[rr[inside],cc[inside]]
    hit = np.zeros(rr.shape,dtype=bool)
    hit[inside] = occluders[rr[inside],cc[inside]]
    unseen = (values < 0) & (np.cumsum(hit,axis=1) == 0)
    scale = max(1,round(.25/grid.resolution))
    bins = np.column_stack((rr[unseen]//scale,cc[unseen]//scale))
    return len(np.unique(bins,axis=0))*(scale*grid.resolution)**2


def surface_weights(grid,surfaces,compact_weight,oriented=False):
    """Balance isolated short objects against dense samples on long walls."""
    weights = np.ones(len(surfaces))
    if compact_weight==1. or not len(surfaces):
        return weights
    occupied = grid.cells >= 65
    labels,_ = ndimage.label(ndimage.binary_dilation(occupied),structure=np.ones((3,3)))
    objects = ndimage.find_objects(labels)
    extents = {}
    for i,surface in enumerate(surfaces):
        r,c = grid.index(*surface[:2])
        if not grid.inside(r,c) or labels[r,c]==0:
            continue
        bounds = objects[labels[r,c]-1]
        extent = [(s.stop-s.start)*grid.resolution for s in bounds]
        if oriented:
            import cv2
            label = int(labels[r,c])
            if label not in extents:
                rows,cols = np.where((labels[bounds]==label) & occupied[bounds])
                points = np.float32(np.column_stack((cols,rows)))*grid.resolution
                extents[label] = cv2.minAreaRect(points)[1]
            extent = extents[label]
        if .3<min(extent) and max(extent)<3.5:
            weights[i] = compact_weight
    return weights


def exploration_utility(goal,robot,distance,gain,visual,visited,mapping_weight,gain_cap):
    bearing = math.atan2(goal[1]-robot[1],goal[0]-robot[0]) if distance>.1 else goal[2]
    initial = math.atan2(math.sin(bearing-robot[2]),math.cos(bearing-robot[2]))
    final = math.atan2(math.sin(goal[2]-bearing),math.cos(goal[2]-bearing))
    turning = abs(initial)+abs(final)
    # One useful nearby face must not lose to overlapping samples on a distant wall.
    reward = mapping_weight*min(gain,gain_cap)+2.0*math.sqrt(min(visual,4))
    revisit = sum(math.exp(-np.linalg.norm(np.asarray(goal[:2])-p)**2/.5) for p in visited)
    return reward/(1.+distance+.35*turning+(.25 if visual else 2.)*revisit)


def confirmed_box_weights(surfaces,groups,available,records,discount):
    """Reduce repeated work on identified objects, without excluding other IDs."""
    weights = np.ones(len(surfaces))
    points = [np.array([r['x'],r['y']]) for r in records if r['confirmed']]
    for group in groups or []:
        # One-sided wall fragments must not suppress nearby wall markers.
        normals = surfaces[group & available,2:]
        if len(normals)<2 or np.min(normals @ normals.T)>.9:
            continue
        if any(np.any(np.linalg.norm(surfaces[group,:2]-p,axis=1)<.45) for p in points):
            weights[group] = np.clip(discount,.05,1.)
    return weights


def coverage_route(goals,visibility,robot,distances,mapping_values,surface_values=None):
    """Select complementary camera views, then shorten an open inspection tour."""
    remaining = np.any(visibility,axis=0)
    selected = []
    for _ in range(min(32,len(goals))):
        marginal = ((visibility & remaining)*(1. if surface_values is None else surface_values)).sum(axis=1)
        score = marginal/(1.+.05*np.asarray(distances))
        k = int(np.argmax(score))
        if marginal[k]==0:
            break
        selected.append(k)
        remaining &= ~visibility[k]
    # Include one useful map-expansion view when visual coverage alone misses it.
    k = int(np.argmax(mapping_values))
    if mapping_values[k]>.5 and not any(np.linalg.norm(np.asarray(goals[k][:2])-goals[i][:2])<2. for i in selected):
        selected.append(k)
    if not selected:
        return [],0.,0.
    poses = np.array([robot[:3],*[goals[i][:3] for i in selected]])
    delta = poses[None,:,:2]-poses[:,None,:2]
    distance = np.linalg.norm(delta,axis=2)
    bearing = np.arctan2(delta[:,:,1],delta[:,:,0])
    bearing = np.where(distance>.05,bearing,poses[None,:,2])
    departure,arrival = bearing-poses[:,None,2],poses[None,:,2]-bearing
    cost = distance+.35*(np.abs(np.arctan2(np.sin(departure),np.cos(departure)))+
                         np.abs(np.arctan2(np.sin(arrival),np.cos(arrival))))
    route,todo = [0],set(range(1,len(poses)))
    while todo:
        k = min(todo,key=lambda i:cost[route[-1],i])
        route.append(k)
        todo.remove(k)
    initial_cost = float(cost[route[:-1],route[1:]].sum())
    best_cost = initial_cost
    for _ in range(2):
        changed = False
        for i in range(1,len(route)-1):
            for j in range(i+1,len(route)):
                candidate = route[:i]+route[i:j+1][::-1]+route[j+1:]
                value = float(cost[candidate[:-1],candidate[1:]].sum())
                if value<best_cost-1e-6:
                    route,best_cost,changed = candidate,value,True
        if not changed:
            break
    return [selected[i-1] for i in route[1:]],initial_cost,best_cost


def compact_box_groups(surfaces,boxes,allow_partial=False):
    """Match box faces to the samples retained after surface deduplication."""
    for faces in boxes:
        # A straight fragment of a wall has no measurable second dimension.
        if not allow_partial and min(np.linalg.norm(faces[i][0]-faces[i+1][0]) for i in (0,2)) <= .30:
            continue
        selected = np.zeros(len(surfaces),dtype=bool)
        for point,normal in faces:
            selected |= (np.linalg.norm(surfaces[:,:2]-point,axis=1)<.60) & (surfaces[:,2:]@normal>.94)
        if selected.any():
            yield selected


def schedule_face_revisit(surfaces,covered,available,ranked,best,state,now,remaining,
                          age_limit=90.,normal_quota=3,cost_aware=False,bounded=False,progress_age=math.inf,
                          revisit_allowed=None,protect_baseline=False):
    """Give pending faces a bounded turn, without target IDs or simulator truth.

    Eligibility is sampled at planning boundaries. Age accrues only when both
    successive samples are eligible. State belongs to the caller, never a worker.
    """
    baseline=best
    state = copy.deepcopy(state or dict(faces=[],next_id=0,normal_choices=0,time=now))
    dt = max(0.,now-state['time'])
    tracks = [t for t in state['faces'] if now-t['seen']<=90.]
    used,matched = set(),[]
    # Global shortest pairs give one-to-one associations; never merge neighbours.
    pairs = sorted((float(np.linalg.norm(face[:2]-t['face'][:2])),i,j)
        for i,face in enumerate(surfaces) for j,t in enumerate(tracks)
        if np.linalg.norm(face[:2]-t['face'][:2])<.35 and np.dot(face[2:],t['face'][2:])>math.cos(math.radians(15)))
    assignments = {}
    for _,i,j in pairs:
        if i not in assignments and j not in used:
            assignments[i]=j;used.add(j)
    previous_active = state.pop('active_id',None)
    for i,face in enumerate(surfaces):
        if i in assignments:
            t = tracks[assignments[i]]
        else:
            t = dict(id=state['next_id'],age=0.,eligible=False,attempts=0,cooldown=0.,seen=now)
            state['next_id']+=1
        eligible = bool(available[i] and not covered[i])
        if eligible and t['eligible'] and t['seen']==state['time']:
            t['age']+=dt
        if covered[i]:
            t.update(age=0.,attempts=0)
        if t['id']==previous_active and not covered[i]:
            # Arriving without camera evidence is an incomplete attempt, not success.
            t['cooldown']=now+(120. if t['attempts']>=2 else 30.)
            if t['attempts']>=2:
                t['attempts']=0
        t.update(face=face.tolist(),seen=now,eligible=eligible)
        matched.append(t)
    stale = [t for j,t in enumerate(tracks) if j not in used]
    for t in stale:
        t['eligible']=False
    state.update(faces=matched+stale,time=now)
    gate=None
    if bounded:
        gate=('warmup' if now<300. else 'recent_marker_progress' if progress_age<90. else
              'baseline_object_priority' if protect_baseline else
              'attempt_limit' if state.get('bounded_count',0)>=2 else
              'spacing' if now-state.get('bounded_last',-math.inf)<90. else None)
    rows,options = [],[]
    for i,t in enumerate(matched):
        candidates = [item for item in ranked if item[5][i]]
        # Respect the existing action's conservative travel allowance, plus stop margin.
        affordable = [item for item in candidates if 20.+item[4]/.10+10.<remaining]
        bounded_candidates=affordable
        if bounded:
            bounded_candidates=[item for item in affordable if best is not None and item[4]<=4.5 and
                item[4]<=best[4]+1.5 and item[0]>=.6*best[0] and
                state.get('bounded_reserved',0.)+30.+item[4]/.10<=90.]
        reason = ('observed' if covered[i] else 'not_reachable' if not available[i] else
                  'no_candidate_or_blocked' if not candidates else 'cooldown' if now<t['cooldown'] else
                  'waiting_age' if t['age']<age_limit else 'time_budget' if not affordable else 'overdue')
        if bounded and reason=='overdue':
            reason=('identified_object' if revisit_allowed is not None and not revisit_allowed[i] else
                    gate or ('baseline_covers_face' if best is not None and best[5][i] else
                    'detour_or_reserve_limit' if not bounded_candidates else 'overdue'))
        rows.append(dict(id=t['id'],face=t['face'],age=round(t['age'],2),reason=reason,
                         path_m=min((item[4] for item in candidates),default=None)))
        if reason=='overdue':
            choice=max(bounded_candidates,key=lambda item:item[0])
            priority=t['age']*max(0.,choice[0]) if cost_aware else t['age']
            rows[-1]['priority']=priority
            options.append((priority,choice[0],t,choice))
    selected=None
    if options and state['normal_choices']>=normal_quota:
        _,_,track,best=max(options,key=lambda item:(item[0],item[1]))
        selected=track['id'];track['attempts']+=1
        state.update(active_id=selected,normal_choices=0)
        if bounded:
            state.update(bounded_count=state.get('bounded_count',0)+1,bounded_last=now,
                         bounded_reserved=state.get('bounded_reserved',0.)+30.+best[4]/.10)
    elif best is not None:
        state['normal_choices']=min(normal_quota,state['normal_choices']+1)
    diagnostic=dict(selected_id=selected,overdue_count=len(options),normal_choices=state['normal_choices'],faces=rows)
    if bounded:
        diagnostic.update(bounded=True,gate=gate,progress_age=progress_age,
                          baseline_goal=list(baseline[1]) if baseline is not None else None,
                          override_count=state.get('bounded_count',0),reserved_seconds=state.get('bounded_reserved',0.))
    return best,state,diagnostic


def continue_object_faces(surfaces,groups,covered,available,ranked,best,state,now,remaining=math.inf):
    """Keep a short commitment to a mapped multi-face object, even at zero coverage."""
    state=copy.deepcopy(state or {})
    candidates=[]
    for group in groups or []:
        faces=surfaces[group & available]
        if len(faces)<2 or np.min(faces[:,2:]@faces[:,2:].T)>.9:
            continue
        center=surfaces[group,:2].mean(axis=0)
        candidates.append((group,center))
    active=None
    if state.get('center') is not None and now<state['until'] and state['steps']<3:
        matches=sorted((float(np.linalg.norm(center-state['center'])),i) for i,(_,center) in enumerate(candidates))
        if matches and matches[0][0]<.65 and (len(matches)<2 or matches[1][0]-matches[0][0]>.15):
            active=candidates[matches[0][1]]
    if active is not None:
        group,center=active
        pending=group & available & ~covered
        # Do not keep commanding an unsuccessful view of the same face.
        for normal in state['attempted_normals']:
            pending &= surfaces[:,2:]@normal<.94
        options=[item for item in ranked if item[4]<=4. and 30.+item[4]/.10<remaining and np.any(item[5]&pending)]
        if options:
            chosen=max(options,key=lambda item:item[0])
            state['steps']+=1
            state['attempted_normals'].extend(surfaces[chosen[5]&pending,2:].tolist())
            return chosen,state,dict(active=True,center=center.tolist(),steps=state['steps'],until=state['until'])
    # After exhausting a commitment, allow a normal goal before starting another.
    if state.get('center') is not None:
        return best,{},dict(active=False,reason='complete_or_budget_or_no_next_face')
    if best is not None and best[1][3] in ('inspect','corner','look') and best[4]<=4. and remaining>45.:
        options=[(group,center) for group,center in candidates if np.any(best[5]&group)]
        if options:
            group,center=min(options,key=lambda pair:np.linalg.norm(pair[1]-best[1][:2]))
            state=dict(center=center.tolist(),until=now+45.,steps=1,
                       attempted_normals=surfaces[best[5]&group,2:].tolist())
            return best,state,dict(active=False,started=True,center=center.tolist(),steps=1,until=state['until'])
    return best,{},dict(active=False,reason='no_object_commitment')


def plan_region_sweep(surfaces,groups,ranked,state,now,remaining,region_size=4.,priority_surfaces=None,refresh_faces=False,face_budget=False):
    """Tour sensor-derived observation regions before returning to serviced ones.

    A service attempt is NOT camera evidence. Persistent object/view cooldowns
    bound retries independently of region switches and moving Hough samples.
    """
    state=copy.deepcopy(state or dict(regions={},objects=[],views=[],active=None))
    state['views']=[v for v in state['views'] if now-v['time']<90.]
    for obj in state['objects']:
        obj['attempts']=[t for t in obj['attempts'] if now-t<120.]
        if face_budget:
            obj['face_attempts']=[f for f in obj.get('face_attempts',[]) if now-f['time']<120.]
    group_objects=[]
    used=set()
    for group in groups or []:
        face=surfaces[group]
        if len(face)<2 or np.min(face[:,2:]@face[:,2:].T)>.9:
            continue
        center=face[:,:2].mean(axis=0)
        matches=[(float(np.linalg.norm(center-o['center'])),j) for j,o in enumerate(state['objects']) if j not in used]
        distance,j=min(matches,default=(math.inf,-1))
        if distance>=.65:
            j=len(state['objects']);state['objects'].append(dict(center=center.tolist(),attempts=[]))
        else:
            state['objects'][j]['center']=center.tolist()
        used.add(j);group_objects.append((group,j))
    object_groups={j:group for group,j in group_objects}
    def new_side(item,j):
        if not face_budget or priority_surfaces is None:
            return False
        normals=surfaces[item[5]&object_groups[j]&priority_surfaces,2:]
        return any(not any(float(n@np.asarray(f['normal']))>.9 for f in state['objects'][j].get('face_attempts',[]))
                   for n in normals)
    options={}; rejected={'view_cooldown':0,'object_cooldown':0,'time_budget':0}
    for item in ranked:
        goal=item[1]
        if 30.+item[4]/.10>=remaining:
            rejected['time_budget']+=1;continue
        if any(np.linalg.norm(np.asarray(goal[:2])-v['pose'][:2])<.65 and
               abs(math.atan2(math.sin(goal[2]-v['pose'][2]),math.cos(goal[2]-v['pose'][2])))<.5
               for v in state['views']):
            rejected['view_cooldown']+=1;continue
        visible=np.flatnonzero(item[5])
        target=(surfaces[min(visible,key=lambda i:np.linalg.norm(surfaces[i,:2]-goal[:2])),:2]
                if len(visible) else np.asarray(goal[:2]))
        objects=[j for group,j in group_objects if np.any(item[5]&group)]
        object_id=min(objects,key=lambda j:np.linalg.norm(np.asarray(state['objects'][j]['center'])-target)) if objects else None
        if object_id is not None and len(state['objects'][object_id]['attempts'])>=3 and not new_side(item,object_id):
            rejected['object_cooldown']+=1;continue
        cell=np.floor(target/region_size).astype(int)
        key=f'{cell[0]},{cell[1]}'
        options.setdefault(key,[]).append((item,object_id))
    diagnostic=dict(regions=[],rejected=rejected,active_region=None)
    if not options:
        return None,state,dict(diagnostic,reason='no_eligible_region')
    refreshed=set()
    if refresh_faces and priority_surfaces is not None:
        # A previously serviced region can gain a newly reachable object side as
        # SLAM expands. Match normals per persistent object, not noisy samples,
        # so unchanged uncovered faces do not reset the tour at every plan.
        additions={}
        for key,items in options.items():
            for item,j in items:
                if j is None:
                    continue
                normals=state['objects'][j].setdefault('available_normals',[])
                for normal in surfaces[item[5]&object_groups[j]&priority_surfaces,2:]:
                    if not any(float(normal@np.asarray(old))>.9 for old in normals):
                        pending=additions.setdefault(j,[])
                        if not any(float(normal@np.asarray(old))>.9 for old in pending):
                            pending.append(normal.tolist())
                        refreshed.add(key)
        for j,normals in additions.items():
            state['objects'][j]['available_normals'].extend(normals)
        for key in refreshed:
            if key in state['regions']:
                state['regions'][key]['visits']=0
    diagnostic['refreshed_regions']=sorted(refreshed)
    for key,items in options.items():
        record=state['regions'].setdefault(key,dict(visits=0,last=-1.))
        diagnostic['regions'].append(dict(key=key,visits=record['visits'],candidates=len(items),
                                         nearest_path=min(t[0][4] for t in items)))
    active=state.get('active')
    keep=active in options and now-state.get('entered',now)<30. and state.get('steps',0)<2
    if not keep:
        choices=[k for k in options if k!=active] or list(options)
        active=min(choices,key=lambda k:(state['regions'][k]['visits'],min(t[0][4] for t in options[k]),
                                         state['regions'][k]['last']))
        state.update(active=active,entered=now,steps=0)
        state['regions'][active]['visits']+=1
    local=options[active]
    priority=([pair for pair in local if np.any(pair[0][5]&priority_surfaces)]
              if priority_surfaces is not None else [])
    chosen,object_id=max(priority or local,key=lambda pair:pair[0][0])
    face_budget_override=bool(object_id is not None and len(state['objects'][object_id]['attempts'])>=3 and new_side(chosen,object_id))
    state['steps']+=1;state['regions'][active]['last']=now
    state['views'].append(dict(pose=list(chosen[1][:3]),time=now))
    if object_id is not None:
        if face_budget:
            ledger=state['objects'][object_id].setdefault('face_attempts',[])
            for normal in surfaces[chosen[5]&object_groups[object_id],2:]:
                ledger.append(dict(normal=normal.tolist(),time=now))
        state['objects'][object_id]['attempts'].append(now)
    diagnostic.update(active_region=active,region_steps=state['steps'],kept_region=keep,
                      object_priority=bool(priority),
                      face_budget_override=face_budget_override,
                      object_id=object_id,object_attempts=len(state['objects'][object_id]['attempts']) if object_id is not None else 0)
    return chosen,state,diagnostic


def plan_mission_goal(grid,robot,clearance,records,visited,blocked,inspected=(),
                      mapping_weight=.4,gain_cap=4.,local_radius=6.,standoff=1.5,max_range=2.2,
                      compact_weight=1.,viewpoint_routing=False,route_commitment=0,pending_route=(),
                      paired_box_views=False,box_view_interval=0,avoid_box_backtracking=False,transit_views=(),
                      nearby_box_inspection=False,frontier_first=False,opportunistic_box_range=0.,frontier_min_gain=.5,
                      object_completion_range=0.,confirmed_box_discount=1.,approach_views=(),observed_surfaces=None,
                      complete_nearby_object=False,global_object_revisit=False,face_revisit=None,object_commitment=None,region_sweep=None,simple_remaining=None,
                      frontier_block_radius=.65,detour_costmap=None,detour_side='either',
                      allow_detour=True,detour_reference_yaw=None,allow_look=True,corridor_forward=False,attempted_backs=()):
    visibility = visible_surfaces if observed_surfaces is None else visible_surface_patches
    reachability = grid.reachable(robot[:2],clearance)
    r,c = grid.index(*robot[:2])
    diagnostic = dict(reachable_cells=int(np.isfinite(reachability[1]).sum()),
        start_meets_clearance=bool(reachability[0][r,c]) if grid.inside(r,c) else False)
    if simple_remaining is not None and detour_costmap is not None:
        slam_goals = np.count_nonzero(reachability[0] & np.isfinite(reachability[1]))
        reachability = nav_goal_reachability(grid,detour_costmap,robot,reachability)
        diagnostic['nav2_reachable_goal_cells'] = int(reachability[0].sum())
        diagnostic['nav2_filtered_goal_cells'] = int(slam_goals - reachability[0].sum())
    for record in sorted(records,key=lambda r:not r['confirmed']):
        if not (record['confirmed'] and record['approached']):
            goal = choose_goal(grid,robot[:2],clearance,blocked=blocked,
                               target=[record['x'],record['y']],reachability=reachability,seen_from=record.get('seen_from'),
                               view_blocked=approach_views)
            if goal is not None:
                diagnostic['approach_target'] = record.get('id')
                return goal,diagnostic
    if corridor_forward and simple_remaining is not None and simple_remaining >= 40.:
        # A turn changes camera yaw but does not reverse the direction of travel.
        past = next((np.array(p[:2]) for p in reversed(transit_views)
                     if np.linalg.norm(np.asarray(robot[:2])-p[:2]) >= .8), None)
        travel_yaw = robot[2] if past is None else math.atan2(robot[1]-past[1], robot[0]-past[0])
        boxes = []
        if allow_look:
            surface_samples(grid, boxes)
        if boxes and past is not None and grid.inside(r,c) and reachability[0][r,c]:
            back = passed_box_face(grid, robot, travel_yaw, transit_views, observed_surfaces,
                                   attempted_backs, max_range, boxes)
            if back is not None:
                center, face = back
                bearing = math.atan2(face[1]-robot[1], face[0]-robot[0])
                diagnostic.update(selection_policy='passed_box_look', box_center=center.tolist(),
                                  goal_surfaces=[face.tolist()])
                return (*robot[:2], bearing, 'look'), diagnostic
        goal = corridor_waypoint(grid, (*robot[:2], travel_yaw), reachability, visited, blocked, frontier_block_radius)
        if goal is not None:
            # Long steps can carry a just-passed object completely out of view.
            # Stop at a useful downstream position; recheck actual evidence there.
            length = np.linalg.norm(np.array(goal[:2])-robot[:2])
            direction = np.array([math.cos(goal[2]), math.sin(goal[2])])
            for along in np.arange(1.2,length,.4) if boxes else ():
                position = np.array(robot[:2]) + along*direction
                if any(np.linalg.norm(position-p) <= frontier_block_radius for p in blocked):
                    continue
                back = passed_box_face(grid, (*position,goal[2]), goal[2],
                                       [*transit_views[-11:],robot], observed_surfaces,
                                       attempted_backs, max_range, boxes)
                if back is not None:
                    goal = (*position, goal[2], 'corridor')
                    diagnostic['downstream_view_of'] = back[0].tolist()
                    break
            diagnostic['selection_policy'] = 'corridor_forward'
            return goal, diagnostic
    if simple_remaining is not None:
        goal,diagnostic = plan_simple_exploration(grid,robot,clearance,visited,blocked,inspected,
            reachability,standoff,max_range,transit_views,observed_surfaces,
            mapping_weight,gain_cap,simple_remaining,diagnostic,frontier_block_radius,allow_look,attempted_backs)
        if goal is None and allow_detour and detour_costmap is not None and simple_remaining >= 20.:
            reference_yaw = robot[2] if detour_reference_yaw is None else detour_reference_yaw
            goal = detour_waypoint(grid,detour_costmap,robot,clearance,visited,blocked,
                                   frontier_block_radius,detour_side,reference_yaw)
            if goal is not None:
                diagnostic['selection_policy'] = 'detour_waypoint'
                diagnostic['detour_reference_yaw'] = float(reference_yaw)
        return goal,diagnostic
    frontiers = frontier_candidates(grid,robot[:2],reachability,blocked,frontier_block_radius)
    opportunistic = None
    if opportunistic_box_range>0:
        groups = []
        opportunistic = inspection_candidates(grid,robot[:2],clearance,inspected,blocked,
            reachability,standoff,max_range,paired_box_views,groups,transit_views,observed_surfaces=observed_surfaces)
        goals,near_surfaces,near_covered,near_available = opportunistic
        if object_completion_range>0:
            # A wall fragment usually offers only its room-facing observation
            # side. Require multiple reachable faces before prioritizing it as
            # an object to circle around.
            groups = [group for group in groups if np.count_nonzero(group & near_available)>=2]
        compact = np.any(groups,axis=0) if groups else np.zeros(len(near_surfaces),dtype=bool)
        near_weights = confirmed_box_weights(near_surfaces,groups,near_available,records,confirmed_box_discount)
        search_range = opportunistic_box_range
        if object_completion_range>opportunistic_box_range:
            occluders = ndimage.binary_dilation(grid.cells>=65)
            if not any(mapping_gain(grid,g[:2],occluders)>=frontier_min_gain for g in frontiers):
                search_range = object_completion_range
        options = []
        for goal in goals:
            r,c = grid.index(*goal[:2])
            distance = float(reachability[1][r,c])
            if distance>search_range:
                continue
            visible = visibility(grid,near_surfaces,goal[:3],max_range) & ~near_covered & compact
            if visible.any():
                utility = exploration_utility(goal,robot,distance,0.,float(near_weights[visible].sum()),visited,0.,0.)
                options.append((utility,goal,distance))
        if options:
            best = max(options,key=lambda item:item[0])
            diagnostic.update(exploration_phase='nearby_object',goal_distance_m=best[2],utility=best[0])
            return best[1],diagnostic
    if frontier_first:
        occluders = ndimage.binary_dilation(grid.cells >= 65)
        ranked_frontiers = []
        for goal in frontiers:
            gain = mapping_gain(grid,goal[:2],occluders)
            if gain < frontier_min_gain:
                continue
            r,c = grid.index(*goal[:2])
            distance = float(reachability[1][r,c])
            utility = exploration_utility(goal,robot,distance,gain,0.,visited,1.,8.)
            ranked_frontiers.append((utility,goal,gain,distance))
        if ranked_frontiers:
            best = max(ranked_frontiers,key=lambda item:item[0])
            diagnostic.update(exploration_phase='frontier',frontier_candidates=len(frontiers),
                              expected_unknown_area_m2=best[2],goal_distance_m=best[3],utility=best[0])
            return best[1],diagnostic
        diagnostic['exploration_phase'] = 'surface_supplement'
    box_groups = [] if box_view_interval or nearby_box_inspection or confirmed_box_discount<1. or object_commitment is not None or region_sweep is not None else None
    inspections,surfaces,covered,available = inspection_candidates(
        grid,robot[:2],clearance,inspected,blocked,reachability,standoff,max_range,paired_box_views,box_groups,transit_views,
        include_thin=frontier_first and opportunistic_box_range<=0,observed_surfaces=observed_surfaces)
    diagnostic.update(frontier_candidates=len(frontiers),surface_candidates=len(inspections),
        reachable_surface_samples=int(available.sum()),observed_surface_samples=int((covered & available).sum()),
        known_free_area_m2=float(np.count_nonzero((grid.cells >= 0) & (grid.cells <= 25))*grid.resolution**2))
    occluders = ndimage.binary_dilation(grid.cells >= 65)
    novelty = confirmed_box_weights(surfaces,box_groups,available,records,confirmed_box_discount)
    weights = surface_weights(grid,surfaces,compact_weight)*novelty
    if viewpoint_routing and route_commitment:
        # Recheck each saved view against the current SLAM map and observations.
        # A saved in-place look becomes a normal travel goal if the robot moved.
        diagnostic['route_tail'] = []
        for i,planned in enumerate(pending_route):
            r,c = grid.index(*planned[:2])
            if not grid.inside(r,c) or not reachability[0][r,c] or not np.isfinite(reachability[1][r,c]):
                continue
            if any(np.linalg.norm(np.asarray(planned[:2])-p)<.65 for p in blocked):
                continue
            if diagnostic['start_meets_clearance'] and np.linalg.norm(np.asarray(planned[:2])-robot[:2])<.25:
                goal = (*robot[:2],planned[2],'look')
            else:
                goal = (*planned[:3],'inspect' if planned[3]=='look' else planned[3])
            gain = 0. if goal[3]=='look' else mapping_gain(grid,goal[:2],occluders)
            visible = visibility(grid,surfaces,goal[:3],max_range) & ~covered & available
            if not visible.any() and gain<.5:
                continue
            diagnostic.update(route_reused=True,route_tail=list(pending_route[i+1:]),
                route_viewpoints=len(pending_route)-i,expected_unknown_area_m2=gain,
                expected_new_surface_samples=int(visible.sum()),goal_distance_m=float(reachability[1][r,c]))
            return goal,diagnostic
    ranked = []
    for goal in inspections+frontiers:
        r,c = grid.index(*goal[:2])
        gain = 0. if goal[3]=='look' else mapping_gain(grid,goal[:2],occluders)
        visible = visibility(grid,surfaces,goal[:3],max_range) & ~covered & available
        visual = float(weights[visible].sum())
        if gain < .5 and visual == 0:
            continue
        distance = float(reachability[1][r,c])
        utility = exploration_utility(goal,robot,distance,gain,visual,visited,mapping_weight,gain_cap)
        ranked.append((utility,goal,gain,visual,distance,visible))
    if ranked:
        if region_sweep is not None:
            priority=(np.any(box_groups,axis=0)&(novelty>=1.) if box_groups and
                      region_sweep.get('object_priority',False) else None)
            chosen,state,details=plan_region_sweep(surfaces,box_groups,ranked,region_sweep['state'],
                region_sweep['now'],region_sweep['remaining'],region_sweep.get('size',4.),priority,
                region_sweep.get('refresh_faces',False),region_sweep.get('face_budget',False))
            diagnostic.update(_region_sweep_state=state,region_sweep=details,selection_policy='region_sweep')
            if chosen is None:
                return None,diagnostic
            diagnostic.update(expected_unknown_area_m2=chosen[2],expected_new_surface_samples=int(chosen[5].sum()),
                              weighted_surface_reward=chosen[3],utility=chosen[0],goal_distance_m=chosen[4])
            return chosen[1],diagnostic
        local = [item for item in ranked if item[4]<=local_radius]
        best = max(local or ranked,key=lambda item:item[0])
        if nearby_box_inspection and box_groups:
            compact = np.any(box_groups,axis=0) & (novelty>=1.)
            box_options = [item for item in local if np.any(item[5] & compact)]
            use_global = False
            if (global_object_revisit and not box_options and
                    not any(item[1][3]=='frontier' and item[2]>=frontier_min_gain for item in ranked)):
                # Once useful map frontiers are exhausted, nearby wall views
                # must not starve an unseen object on the other side of the room.
                box_options = [item for item in ranked if np.any(item[5] & compact)]
                use_global = bool(box_options)
            recent_box_views = 0
            for pose in reversed(inspected[-2:]):
                if not np.any(visibility(grid,surfaces,pose,max_range) & compact):
                    break
                recent_box_views += 1
            # Two nearby box views, then give walls/frontiers a turn. This uses
            # only mapped obstacle geometry, never target IDs or scene positions.
            if recent_box_views<2 and box_options:
                looks = [item for item in box_options if item[1][3]=='look']
                best = max(looks or box_options,key=lambda item:item[0])
                diagnostic['nearby_box_priority'] = 'look' if looks else 'face'
                if use_global:
                    diagnostic['global_object_revisit'] = True
            elif recent_box_views>=2:
                other = [item for item in local if not np.any(item[5] & compact)]
                if other:
                    best = max(other,key=lambda item:item[0])
                    diagnostic['nearby_box_priority'] = 'wall_or_frontier_turn'
            diagnostic['recent_box_views'] = recent_box_views
        if complete_nearby_object and observed_surfaces is not None and box_groups:
            # Finish reachable sides of the object just observed before a long
            # detour. A nearby parallel wall fragment is not a multi-face object.
            recent = visibility(grid,surfaces,robot,max_range)
            if inspected:
                recent |= visibility(grid,surfaces,inspected[-1],max_range)
            unfinished = np.zeros(len(surfaces),dtype=bool)
            for group in box_groups:
                normals = surfaces[group & available,2:]
                if len(normals)<2 or np.min(normals@normals.T)>.9:
                    continue
                if not np.any(group & covered & recent) or not np.any(group & ~covered & available):
                    continue
                if np.min(np.linalg.norm(surfaces[group,:2]-robot[:2],axis=1))>2.8:
                    continue
                unfinished |= group & (novelty>=1.)
            completion = [item for item in ranked if item[4]<=4. and np.any(item[5] & unfinished)]
            if completion:
                best = max(completion,key=lambda item:item[0])
                diagnostic['object_completion_priority'] = True
        if box_view_interval and box_groups and inspected and not viewpoint_routing and best[1][3]!='frontier':
            compact = np.any(box_groups,axis=0)
            alternatives = [item for item in local if np.any(item[5] & compact)]
            if alternatives and avoid_box_backtracking:
                # Infer the last direction of travel from positions, not camera yaw.
                for previous in reversed(visited):
                    displacement = np.asarray(robot[:2])-previous
                    length = np.linalg.norm(displacement)
                    if length>=1.:
                        alternatives = [item for item in alternatives
                            if np.dot(np.asarray(item[1][:2])-robot[:2],displacement)>=-.75*length]
                        break
            same_box = np.zeros(len(surfaces),dtype=bool)
            if np.linalg.norm(np.asarray(inspected[-1][:2])-robot[:2])<=.6:
                latest = visibility(grid,surfaces,inspected[-1],max_range)
                for group in box_groups:
                    if np.any(latest & group):
                        same_box |= group
            nearby_faces = [item for item in alternatives if item[4]<=3.5 and np.any(item[5] & same_box)]
            if nearby_faces and not np.any(best[5] & same_box):
                best = max(nearby_faces,key=lambda item:item[0])
                diagnostic['box_view_priority'] = 'nearby_face'
            elif alternatives and len(inspected)>=box_view_interval and not np.any(best[5] & compact):
                # Strip faces already covered before the recent views.
                recently_covered = surfaces[covered & compact]
                for pose in inspected[:-box_view_interval]:
                    recently_covered = recently_covered[~visibility(grid,recently_covered,pose,max_range)]
                    if not len(recently_covered):
                        break
                if not len(recently_covered):
                    best = max(alternatives,key=lambda item:item[0])
                    diagnostic['box_view_priority'] = 'observation_gap'
        if viewpoint_routing:
            route,initial,cost = coverage_route([item[1] for item in ranked],
                np.array([item[5] for item in ranked]),robot,[item[4] for item in ranked],
                [mapping_weight*min(item[2],gain_cap) for item in ranked],novelty)
            if route:
                best = ranked[route[0]]
                if route_commitment:
                    diagnostic['route_tail'] = [ranked[i][1] for i in route[1:route_commitment]]
            diagnostic.update(route_viewpoints=len(route),route_initial_cost=initial,route_cost=cost)
        if face_revisit is not None:
            compact_pending=(np.any(box_groups,axis=0)&(novelty>=1.) if box_groups else np.zeros(len(surfaces),dtype=bool))
            best,state,details = schedule_face_revisit(surfaces,covered,available,ranked,best,
                face_revisit['state'],face_revisit['now'],face_revisit['remaining'],
                face_revisit.get('age',90.),face_revisit.get('quota',3),face_revisit.get('cost_aware',False),
                face_revisit.get('bounded',False),face_revisit.get('progress_age',math.inf),
                novelty>=1.,bool(np.any(best[5]&compact_pending)))
            diagnostic.update(_face_revisit_state=state,face_revisit=details)
            if details['selected_id'] is not None:
                diagnostic['selection_policy']='overdue_face'
                diagnostic.pop('route_tail',None)
        if object_commitment is not None:
            best,state,details=continue_object_faces(surfaces,box_groups,covered,available,ranked,best,
                object_commitment['state'],object_commitment['now'],object_commitment['remaining'])
            diagnostic.update(_object_commitment_state=state,object_commitment=details)
            if details['active']:
                diagnostic['selection_policy']='object_face_commitment'
                diagnostic.pop('route_tail',None)
        count = (visibility(grid,surfaces,best[1][:3],max_range) & ~covered & available).sum()
        diagnostic.update(expected_unknown_area_m2=best[2],expected_new_surface_samples=int(count),
                          weighted_surface_reward=best[3],utility=best[0])
        diagnostic.update(goal_distance_m=best[4],local_candidates=len(local))
        return best[1],diagnostic
    if face_revisit is not None:
        _,state,details = schedule_face_revisit(surfaces,covered,available,[],None,
            face_revisit['state'],face_revisit['now'],face_revisit['remaining'],
            face_revisit.get('age',90.),face_revisit.get('quota',3),face_revisit.get('cost_aware',False),
            face_revisit.get('bounded',False),face_revisit.get('progress_age',math.inf))
        diagnostic.update(_face_revisit_state=state,face_revisit=details)
    if object_commitment is not None:
        diagnostic['_object_commitment_state']={}
    return None,diagnostic


@dataclass
class Track:
    samples: deque = field(default_factory=lambda: deque(maxlen=20))
    last_stamp: float = -1.0
    confirmed: bool = False
    approached: bool = False
    observer: object = None
    alternatives: deque = field(default_factory=lambda: deque(maxlen=10))


class TargetRegistry:
    def __init__(self, min_hits=5, gate=.45, max_spread=.2, window=5.0, association_radius=.35):
        self.tracks = {}
        self.min_hits, self.gate = min_hits, gate
        self.max_spread, self.window = max_spread, window
        self.association_radius = association_radius

    def add(self, marker_id, point, stamp, robot):
        point = np.asarray(point,dtype=float)[:2]
        if not np.all(np.isfinite(point)):
            return False
        track = self.tracks.setdefault(int(marker_id),Track())
        if stamp <= track.last_stamp:
            return False
        if track.samples and np.linalg.norm(point-np.median([p for _,p in track.samples],axis=0)) > self.gate:
            if track.confirmed:
                if track.alternatives and stamp <= track.alternatives[-1][0]:
                    return False
                while track.alternatives and stamp-track.alternatives[0][0] > self.window:
                    track.alternatives.popleft()
                track.alternatives.append((stamp,point))
                positions = np.array([p for _,p in track.alternatives])
                center = np.median(positions,axis=0)
                if len(positions) < self.min_hits or np.quantile(np.linalg.norm(positions-center,axis=1),.8) > self.max_spread:
                    return False
                track.samples.clear()
                track.samples.extend(track.alternatives)
                track.alternatives.clear()
            elif stamp-track.last_stamp < self.window:
                return False
            else:
                track.samples.clear()
        else:
            track.alternatives.clear()
        track.last_stamp = stamp
        track.observer = np.asarray(robot,dtype=float)[:2].tolist()
        if not track.confirmed:
            while track.samples and stamp-track.samples[0][0] > self.window:
                track.samples.popleft()
        if not track.samples or track.samples[-1][0] != stamp:
            track.samples.append((stamp,point))
        samples = np.array([p for _,p in track.samples])
        center = np.median(samples,axis=0)
        spread = np.linalg.norm(samples-center,axis=1)
        if len(samples) >= self.min_hits and np.quantile(spread,.8) <= self.max_spread:
            track.confirmed = True
        # Only credit approach when marker is actually observed, not through a wall.
        if track.confirmed and np.linalg.norm(point-np.asarray(robot)[:2]) <= 1.9:
            track.approached = True
        return True

    def records(self):
        result = []
        for marker_id,track in sorted(self.tracks.items()):
            if not track.samples:
                continue
            center = np.median([p for _,p in track.samples],axis=0)
            result.append(dict(id=marker_id,x=float(center[0]),y=float(center[1]),
                               confirmed=track.confirmed,approached=track.approached,
                               hits=len(track.samples),last_stamp=track.last_stamp,seen_from=track.observer))
        # Distinct IDs at the same physical position must not count as two targets.
        # Prefer the hypothesis with more supporting frames; never blacklist a specific ID.
        unique = []
        for record in sorted(result,key=lambda r:(-r['hits'],-int(r['confirmed']),r['id'])):
            if any(math.hypot(record['x']-other['x'],record['y']-other['y']) < self.association_radius
                   for other in unique):
                continue
            unique.append(record)
        return sorted(unique,key=lambda r:r['id'])


def passed_box_face(grid, robot, yaw, transit_views, observations, attempted, max_range, boxes):
    """Select one visible downstream face of a compact object crossed on this pass."""
    forward = np.array([math.cos(yaw), math.sin(yaw)])
    position = np.asarray(robot[:2])
    trail = np.array([p[:2] for p in transit_views[-12:]])
    best = None
    best_distance = max_range
    for box in boxes:
        faces = np.array([[*point, *normal] for point, normal in box])
        center = faces[:, :2].mean(axis=0)
        if any(np.linalg.norm(center - previous[:2]) < .65 for previous in attempted):
            continue
        delta = center - position
        if delta @ forward >= -.3 or np.linalg.norm(delta) > max_range + 1.:
            continue
        # There must be a recent camera position on the upstream side of this object.
        if not np.any(((trail - center) @ forward < -.3) &
                      (np.linalg.norm(trail - center, axis=1) < max_range)):
            continue
        faces = faces[faces[:, 2:] @ forward > .65]
        if observations is not None:
            faces = faces[~observed_surface_mask(faces, observations)]
        for face in faces:
            if surface_weights(grid, face[None,:], 2., oriented=True)[0] <= 1.:
                continue
            offset = face[:2] - position
            bearing = math.atan2(offset[1], offset[0])
            distance = np.linalg.norm(offset)
            if distance < best_distance and visible_surface_patches(
                    grid, face[None,:], (*position, bearing), max_range)[0]:
                best, best_distance = (center, face), distance
    return best


def corridor_waypoint(grid, robot, reachability, visited, blocked, blocked_radius):
    """Follow local wall axes as far as the mapped, navigable strip permits."""
    import cv2
    origin = np.asarray(robot[:2])
    safe, distances = reachability
    step = grid.resolution / 2
    row, col = grid.index(*origin)
    radius = math.ceil(4. / grid.resolution)
    local = grid.cells[max(0,row-radius):row+radius+1, max(0,col-radius):col+radius+1]
    if not local.size:
        return None
    lines = cv2.HoughLinesP((local >= 65).astype(np.uint8), 1, math.pi/180,
                            threshold=max(5,round(1./grid.resolution)),
                            minLineLength=1.5/grid.resolution, maxLineGap=.2/grid.resolution)
    if lines is None:
        return None
    headings = []
    for x1,y1,x2,y2 in lines[:,0]:
        yaw = math.atan2(y2-y1,x2-x1) + grid.origin[2]
        yaw = robot[2] + .5*math.atan2(math.sin(2*(yaw-robot[2])), math.cos(2*(yaw-robot[2])))
        if all(abs(yaw-other) > math.radians(5) for other in headings):
            headings.append(yaw)
    for yaw in sorted(headings, key=lambda angle: abs(angle-robot[2])):
        forward = np.array([math.cos(yaw), math.sin(yaw)])
        lateral = np.array([-forward[1], forward[0]])
        clear = 0.
        for distance in np.arange(step, 3.5 + .001, step):
            cell = grid.index(*(origin + distance*forward))
            if not grid.inside(*cell) or not safe[cell] or not np.isfinite(distances[cell]):
                break
            clear = distance
        if clear < 1.2:
            continue
        end = origin + clear*forward
        if any(np.linalg.norm(end-point) < .8 for point in visited):
            continue
        if any(np.linalg.norm(end-point) <= blocked_radius for point in blocked):
            continue
        widths = []
        for along in np.linspace(0, clear, 7):
            sides = []
            for sign in (-1, 1):
                for distance in np.arange(step, 4. + step, step):
                    cell = grid.index(*(origin + along*forward + sign*distance*lateral))
                    if not grid.inside(*cell) or not 0 <= grid.cells[cell] <= 25:
                        if grid.inside(*cell) and grid.cells[cell] >= 65:
                            sides.append(distance)
                        break
            if len(sides) == 2 and sum(sides) <= 4.:
                widths.append(sides)
        if len(widths) >= 4:
            widths = np.asarray(widths)
            # Doorways and small objects can interrupt either mapped wall.
            consistent = np.max(np.abs(widths - np.median(widths, axis=0)), axis=1) <= .35
            if np.count_nonzero(consistent) >= 4:
                return float(end[0]), float(end[1]), float(yaw), 'corridor'
    return None


def plan_simple_exploration(grid,robot,clearance,visited,blocked,inspected,
                            reachability,standoff,max_range,transit_views,observed_surfaces,
                            mapping_weight,gain_cap,remaining,diagnostic,frontier_block_radius=.65,allow_look=True,attempted_backs=()):
    """One global ranking; only actual camera evidence retires surface patches."""
    inspections,surfaces,covered,available=inspection_candidates(
        grid,robot[:2],clearance,inspected,blocked,reachability,standoff,max_range,
        paired_box_views=True,transit_views=transit_views,observed_surfaces=observed_surfaces)
    attempted_faces = [attempt[2:] for attempt in attempted_backs]
    if attempted_faces:
        available &= ~observed_surface_mask(surfaces, attempted_faces)
    frontiers=frontier_candidates(grid,robot[:2],reachability,blocked,frontier_block_radius)
    occluders=ndimage.binary_dilation(grid.cells>=65)
    compact=surface_weights(grid,surfaces,3.,oriented=True)>1.
    # Spatial novelty is measured against camera-covered wall patches, not
    # robot visits, map exploration, elapsed time, or simulator truth.
    wall_novelty=np.ones(len(surfaces))
    seen_walls=surfaces[covered & ~compact]
    if len(seen_walls):
        separation=np.linalg.norm(surfaces[:,None,:2]-seen_walls[None,:,:2],axis=2)
        separation[surfaces[:,2:]@seen_walls[:,2:].T<.94]=np.inf
        wall_novelty=np.minimum(separation.min(axis=1),3.)/3.
    options=[]
    out_of_time=0
    for goal in inspections+frontiers:
        if goal[3] == 'look' and not allow_look:
            continue
        r,c=grid.index(*goal[:2])
        distance=float(reachability[1][r,c])
        bearing=math.atan2(goal[1]-robot[1],goal[0]-robot[0]) if distance>.1 else goal[2]
        turning=abs(math.atan2(math.sin(bearing-robot[2]),math.cos(bearing-robot[2])))
        turning+=abs(math.atan2(math.sin(goal[2]-bearing),math.cos(goal[2]-bearing)))
        cost=8.+distance/.15+turning/.5
        if cost+10.>remaining:
            out_of_time+=1
            continue
        visible=visible_surface_patches(grid,surfaces,goal[:3],max_range)&~covered&available
        gain=0. if goal[3]=='look' else mapping_gain(grid,goal[:2],occluders)
        if not visible.any() and gain<.5:
            continue
        # Wall sample density must not outweigh an unseen small-object face.
        # Only camera-unobserved samples earn reward; covered faces never do.
        wall_views=int(np.any(visible & ~compact))
        object_faces=int(np.count_nonzero(visible & compact))
        novelty=float(wall_novelty[visible & ~compact].max()) if wall_views else 0.
        visual_reward=2.*wall_views+6.*math.sqrt(min(object_faces,2))
        reward=visual_reward+mapping_weight*min(gain,gain_cap)
        repeats=sum(math.exp(-np.linalg.norm(np.asarray(goal[:2])-p)**2/.5) for p in visited)
        score=reward/(cost+5.*repeats)+.09*novelty
        options.append((score,goal,gain,visible,distance,cost,wall_views,object_faces,visual_reward,novelty))
    diagnostic.update(selection_policy='simple_global',score_version='wall_region_utility',frontier_candidates=len(frontiers),
        surface_candidates=len(inspections),feasible_candidates=len(options),out_of_time_candidates=out_of_time,
        reachable_surface_samples=int(available.sum()),observed_surface_samples=int((covered&available).sum()))
    if not options:
        return None,diagnostic
    score,goal,gain,visible,distance,cost,wall_views,object_faces,visual_reward,novelty=max(options,key=lambda item:item[0])
    diagnostic.update(utility=score,expected_unknown_area_m2=gain,
        expected_new_surface_samples=int(visible.sum()),goal_distance_m=distance,
        estimated_seconds=cost,wall_region_novelty=novelty,wall_views=wall_views,object_faces=object_faces,visual_reward=visual_reward,
        goal_surfaces=surfaces[visible].tolist())
    return goal,diagnostic
