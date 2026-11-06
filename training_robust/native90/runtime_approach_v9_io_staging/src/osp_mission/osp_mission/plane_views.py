"""Generate viewpoints from observed straight faces, independent of map tiles."""

import math

import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt, label, uniform_filter
from scipy.spatial import cKDTree


def line_anchors(points, spacing=1.4):
    """Symmetric sampling of a fitted line; eigenvector sign does not matter."""
    centre=points.mean(axis=0)
    if len(points)<2:return centre[None,:],np.array([0.,1.]),0.
    delta=points-centre
    _,vectors=np.linalg.eigh(delta.T@delta)
    tangent=vectors[:,-1]
    projections=delta@tangent;low=float(projections.min());high=float(projections.max())
    span=high-low
    count=max(1,int(math.ceil(span/spacing-1e-9)))
    samples=low+(np.arange(count)+.5)*span/count
    return centre+samples[:,None]*tangent,tangent,span


def nearest_ties(tree, coordinates, targets, maximum=.25):
    """Preserve tied nearest cells instead of depending on KD-tree ordering."""
    if not len(targets) or not len(coordinates):return []
    distances,indices=tree.query(targets,k=min(4,len(coordinates)))
    distances=np.asarray(distances).reshape(len(targets),-1)
    indices=np.asarray(indices).reshape(len(targets),-1)
    result=[]
    for d,index in zip(distances,indices):
        if d[0]>maximum+1e-9:continue
        result.extend(int(i) for i in index[d<=d[0]+1e-9])
    return result


def sample(data,resolution,origin,robot,safe,distances,faces,viewed,include_curved=False):
    """Return reachable cells in front of known faces and at known frontiers."""
    usable=safe&np.isfinite(distances)&(distances>.45)
    usable &= uniform_filter((data==0).astype(float),size=3)>.65
    cells=np.column_stack(np.where(usable))
    if not len(cells):return []
    coordinates=(cells+.5)*resolution  # y,x; origin cancels for local geometry.
    tree=cKDTree(coordinates);points=set()
    angles=np.arctan2(faces['ny'],faces['nx'])
    bins=np.floor((angles+math.pi/16)/(math.pi/8)+1e-10).astype(int)%16
    inspected=viewed[faces['side'],faces['y'],faces['x']]
    for direction in range(16):
        members=np.flatnonzero(bins==direction)
        if not len(members):continue
        mask=np.zeros(data.shape,dtype=bool)
        mask[faces['y'][members],faces['x'][members]]=True
        groups,_=label(mask,structure=np.ones((3,3)))
        identifiers=groups[faces['y'][members],faces['x'][members]]
        for ident in np.unique(identifiers):
            indices=members[identifiers==ident]
            if inspected[indices].all():continue
            positions=np.column_stack((faces['py'][indices],faces['px'][indices]))*resolution
            anchors,tangent,span=line_anchors(positions)
            mean=np.array([faces['ny'][indices].mean(),faces['nx'][indices].mean()])
            if span<.15-1e-9:
                if not include_curved:continue
                # Short arcs and small planar faces still support a marker.
                # Their local normal is more stable than a short line fit.
                normal=mean/max(float(np.linalg.norm(mean)),1e-9)
                anchors=positions.mean(axis=0)[None,:]
            else:
                normal=np.array([-tangent[1],tangent[0]])
                if normal@mean<0:normal=-normal
            targets=np.concatenate([anchors+stand*normal for stand in (.6,1.2,2.2)])
            for index in nearest_ties(tree,coordinates,targets):points.add(tuple(cells[index]))
    # Frontier locations also follow observed components rather than fixed tiles.
    frontier=usable&binary_dilation(data<0)
    groups,count=label(frontier,structure=np.ones((3,3)))
    for ident in range(1,count+1):
        local_cells=np.column_stack(np.where(groups==ident))
        if len(local_cells)<2:continue
        local=(local_cells+.5)*resolution
        anchors,_,_=line_anchors(local,spacing=1.5)
        local_tree=cKDTree(local)
        for index in nearest_ties(local_tree,local,anchors,maximum=float('inf')):
            points.add(tuple(local_cells[index]))
    return sorted((int(y),int(x)) for y,x in points)


def sample_all(data,resolution,origin,robot,safe,distances,faces,viewed):
    return sample(data,resolution,origin,robot,safe,distances,faces,viewed,include_curved=True)


def symmetric_grid(data,resolution,origin,robot,safe,distances,faces,viewed):
    """Cover free space with tiles centred on its observed extent, keeping ties."""
    clearance=distance_transform_edt(data<=50)*resolution
    gain=uniform_filter((data<0).astype(float),size=max(3,int(3/resolution)|1))
    valid=safe&np.isfinite(distances)&(distances>.45)&((clearance<3.2)|(gain>.015))
    valid &= uniform_filter((data==0).astype(float),size=3)>.65
    cells=np.column_stack(np.where(valid))
    if not len(cells):return []
    low=cells.min(axis=0);span=cells.max(axis=0)+1-low
    bins=np.maximum(1,np.ceil(span*resolution-1e-9).astype(int))
    width=span/bins;positions=cells+.5;points=set()
    scores=np.abs(clearance[cells[:,0],cells[:,1]]-2.)
    relative=(positions-low)/width
    main=np.floor(relative).astype(int)
    nearest=np.rint(relative).astype(int)
    boundary=(np.abs(positions-(low+nearest*width))<=1e-9)&(nearest>0)&(nearest<bins)
    extra=np.where(boundary,np.where(main==nearest,nearest-1,nearest),-1)
    assignments=[];indices=[]
    for rows,cols in [(main[:,0],main[:,1]),(extra[:,0],main[:,1]),
                      (main[:,0],extra[:,1]),(extra[:,0],extra[:,1])]:
        valid=(rows>=0)&(cols>=0)
        assignments.extend((rows[valid]*bins[1]+cols[valid]).tolist())
        indices.extend(np.flatnonzero(valid).tolist())
    assignments=np.asarray(assignments);indices=np.asarray(indices)
    order=np.argsort(assignments,kind='stable');ordered=assignments[order]
    for group in np.split(order,np.flatnonzero(np.diff(ordered))+1):
        ident=int(assignments[group[0]]);row,col=divmod(ident,int(bins[1]))
        centre=low+(np.array([row,col])+.5)*width
        local=indices[group];local=local[scores[local]<=scores[local].min()+1e-9]
        proximity=((positions[local]-centre)**2).sum(axis=1)
        for index in local[proximity<=proximity.min()+1e-9]:points.add(tuple(cells[index]))
    return sorted((int(y),int(x)) for y,x in points)


def hybrid(data,resolution,origin,robot,safe,distances,faces,viewed):
    arguments=(data,resolution,origin,robot,safe,distances,faces,viewed)
    return sorted(set(symmetric_grid(*arguments))|set(sample_all(*arguments)))
