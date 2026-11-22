"""Finalize confirmed landmark coordinates against the saved SLAM pose graph.

Only detector observations and SLAM's own corrected/odometric scan poses enter
this calculation. Independent scenario geometry and truth are never inputs.
"""
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import time

import numpy as np


def read_graph(path):
    keys=('node_id','stamp_sim_s','odom_x','odom_y','odom_yaw','map_x','map_y','map_yaw')
    with path.open() as stream:
        values=np.asarray([[float(row[key]) for key in keys] for row in csv.DictReader(stream)],dtype=float)
    if values.ndim!=2 or values.shape[1]!=8 or not len(values) or not np.isfinite(values).all():
        raise ValueError('Require finite serialized SLAM scan poses')
    values=values[np.argsort(values[:,1],kind='stable')]
    return values[np.r_[np.diff(values[:,1])>0,True]]


def finalize(observations, graph, confirmed_targets):
    graph=np.asarray(graph,dtype=float)
    if graph.ndim!=2 or graph.shape[1]!=8 or not len(graph) or not np.isfinite(graph).all():
        raise ValueError('Invalid graph')
    if np.any(np.diff(graph[:,1])<=0):raise ValueError('Require strictly increasing graph stamps')
    final={int(key):tuple(value) for key,value in confirmed_targets.items()}
    grouped={ident:[] for ident in final}
    rejected={}
    for value in sorted(observations,key=lambda row:row.get('stamp',-math.inf)):
        try:
            ident=int(value['id']);stamp=float(value['stamp']);point=np.asarray(value.get('odom_point'),dtype=float)
            pixels=float(value.get('min_edge_pixels',0))
            if (ident not in grouped or value.get('provisional') or point.shape!=(3,)
                    or not np.isfinite(point).all() or not np.isfinite([stamp,pixels]).all() or pixels<18):continue
            if not graph[0,1]<=stamp<=graph[-1,1]:continue
            index=int(np.searchsorted(graph[:,1],stamp))
            candidates=[i for i in (index-1,index) if 0<=i<len(graph)]
            scan=graph[min(candidates,key=lambda i:abs(graph[i,1]-stamp))]
            if abs(scan[1]-stamp)>.35+1e-9:
                rejected['scan_time_gap']=rejected.get('scan_time_gap',0)+1;continue
            rows=grouped[ident]
            if rows and stamp-rows[-1]['stamp']<.15-1e-9:continue
            angle=scan[7]-scan[4];c,s=math.cos(angle),math.sin(angle)
            relative=point[:2]-scan[2:4]
            mapped=scan[5:7]+np.array([[c,-s],[s,c]])@relative
            rows.append(dict(stamp=stamp,node_id=int(scan[0]),scan_stamp=float(scan[1]),point=mapped.tolist()))
        except (ValueError,TypeError,KeyError,IndexError):
            rejected['malformed_observation']=rejected.get('malformed_observation',0)+1
    reports=[]
    for ident,rows in grouped.items():
        record=dict(id=ident,original_point=list(final[ident]),eligible_observations=len(rows),applied=False)
        if rows:
            values=np.asarray([row['point'] for row in rows]);center=np.median(values,axis=0)
            inliers=[row for row in rows if np.linalg.norm(np.asarray(row['point'])-center)<=.25+1e-9]
            nodes={row['node_id'] for row in inliers}
            record.update(inlier_observations=len(inliers),distinct_scan_nodes=len(nodes))
            if len(inliers)>=3 and len(nodes)>=3:
                point=np.median([row['point'] for row in inliers],axis=0)
                record.update(applied=True,point=point.tolist(),shift_m=float(np.linalg.norm(point-final[ident])),
                    observations=inliers)
                final[ident]=tuple(map(float,point))
            else:record['reason']='insufficient_consistent_frames_or_distinct_scan_nodes'
        else:record['reason']='no_time_matched_valid_odom_observations'
        reports.append(record)
    return final,dict(method='confirmed_odom_landmarks_reprojected_by_final_SLAM_scan_poses',
        ground_truth_inputs=False,required_readable_frames=3,required_distinct_scan_nodes=3,
        maximum_scan_time_gap_sim_s=.35,maximum_inlier_deviation_m=.25,
        corrected_ids=[row['id'] for row in reports if row['applied']],rows=reports,rejected=rejected)


def finalize_output(observations, targets, output, exporter):
    """Only replace final landmark CSV after a successful graph-only calculation."""
    started=time.monotonic();report=dict(applied=False,ground_truth_inputs=False,corrected_ids=[])
    original={int(key):tuple(value) for key,value in targets.items()}
    try:
        output=Path(output);graph=output/'graph_scan_poses.csv';data=output/'map.data'
        if not data.is_file():raise ValueError('Serialized SLAM dataset unavailable')
        (output/'graph_marker_observations.json').write_text(json.dumps(observations,allow_nan=False)+'\n')
        with (output/'graph_export.log').open('w') as stream:
            subprocess.run([str(exporter),str(data),str(graph)],stdout=stream,stderr=stream,check=True,timeout=5)
        final,report=finalize(observations,read_graph(graph),original)
        report['applied']=bool(report['corrected_ids'])
        report['input_sha256']={path.name:hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (data,graph,output/'graph_marker_observations.json')}
        if report['applied']:
            shutil.copy2(output/'markers.csv',output/'markers_before_graph.csv')
            temporary=output/'markers.graph.tmp'
            with temporary.open('w') as stream:
                writer=csv.writer(stream);writer.writerow(['id','x','y'])
                for ident,(x,y) in sorted(final.items()):writer.writerow([ident,x,y])
            temporary.replace(output/'markers.csv')
        else:final=original
    except Exception as exc:
        final=original;report.update(applied=False,corrected_ids=[],error=repr(exc),original_coordinates_preserved=True)
    report['finalization_wall_seconds']=time.monotonic()-started
    report['driving_time_and_approach_credits_modified']=False
    (Path(output)/'graph_landmark_update.json').write_text(json.dumps(report,allow_nan=False,indent=2)+'\n')
    return final,report
