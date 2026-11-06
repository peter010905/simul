"""Associate measured LiDAR endpoints with quantized SLAM surface cells."""
import numpy as np


def match(data, resolution, origin, endpoints):
    points=np.asarray(endpoints,dtype=float).reshape(-1,2)
    points=points[np.isfinite(points).all(axis=1)]
    columns=np.floor((points[:,0]-origin[0])/resolution).astype(int)
    rows=np.floor((points[:,1]-origin[1])/resolution).astype(int)
    inside=(rows>=0)&(rows<data.shape[0])&(columns>=0)&(columns<data.shape[1])
    points,rows,columns=points[inside],rows[inside],columns[inside]
    matched_rows=rows.copy();matched_columns=columns.copy()
    exact=data[rows,columns]>50
    best=np.where(exact,-1.,(1.25*resolution)**2)
    matched=exact.copy()
    # The raw range lies on a physical boundary, whereas map occupancy is
    # quantized. Permit one neighbouring cell, within a fixed metric bound.
    # Distant hits cannot inspect a surface merely because it is in the FOV.
    for dy in (-1,0,1):
        for dx in (-1,0,1):
            y=rows+dy;x=columns+dx
            valid=(y>=0)&(y<data.shape[0])&(x>=0)&(x<data.shape[1])
            yc=y.clip(0,data.shape[0]-1);xc=x.clip(0,data.shape[1]-1)
            error=(origin[0]+(x+.5)*resolution-points[:,0])**2+(origin[1]+(y+.5)*resolution-points[:,1])**2
            select=valid&(data[yc,xc]>50)&(error<best)
            matched_rows[select]=y[select];matched_columns[select]=x[select]
            best[select]=error[select];matched|=select
    return points[matched],matched_rows[matched],matched_columns[matched]
