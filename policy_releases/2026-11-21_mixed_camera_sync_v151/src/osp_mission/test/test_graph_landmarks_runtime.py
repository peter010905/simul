"""Independent geometric and evidence guard contracts for final map updates."""
import copy
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest

try:
    from osp_mission.graph_landmarks import finalize,read_graph,finalize_output
except ImportError:
    sys.path.insert(0,str(Path(__file__).resolve().parent))
    from candidate_graph_landmarks import finalize,read_graph,finalize_output


def inputs():
    # Analytic map<-odom transform: rotate pi/2,translate (2,3).
    graph=np.array([[i,t,1.,0.,0.,2.,4.,math.pi/2] for i,t in enumerate([1.,1.2,1.4,1.6,1.8])])
    rows=[dict(id=7,stamp=t,odom_point=[2.,0.,.5],min_edge_pixels=24,map_point=[999.,999.,999.])
          for t in [1.,1.2,1.4]]
    return rows,graph,{7:(-10.,-10.)}


def test_reprojects_with_scan_odom_relative_pose_and_rotation_without_using_recorded_map_point():
    rows,graph,original=inputs();before=copy.deepcopy(rows)
    result,report=finalize(rows,graph,original)
    assert result[7]==pytest.approx((2.,5.))
    assert report['corrected_ids']==[7]
    assert rows==before and original=={7:(-10.,-10.)}
    assert not report['ground_truth_inputs']


@pytest.mark.parametrize('change', ['unconfirmed','provisional','small','missing_odom','nonfinite','same_scan','duplicate','time_gap'])
def test_bad_or_insufficient_evidence_keeps_original_coordinate(change):
    rows,graph,original=inputs()
    if change=='unconfirmed':original={8:(-10.,-10.)}
    elif change=='provisional':rows[0]['provisional']=True
    elif change=='small':rows[0]['min_edge_pixels']=17.99
    elif change=='missing_odom':rows[0]['odom_point']=None
    elif change=='nonfinite':rows[0]['odom_point'][0]=float('nan')
    elif change=='same_scan':graph=graph[[0,-1]]
    elif change=='duplicate':rows[1]['stamp']=rows[0]['stamp']
    elif change=='time_gap':graph=np.array([[0,0.,0,0,0,0,0,0],[1,2.,0,0,0,0,0,0]])
    result,report=finalize(rows,graph,original)
    assert result==original and report['corrected_ids']==[]


def test_sparse_bad_projection_cannot_move_readable_consensus():
    rows,graph,original=inputs()
    rows.append(dict(rows[-1],stamp=1.6,odom_point=[10.,0.,.5]))
    result,report=finalize(rows,graph,original)
    assert result[7]==pytest.approx((2.,5.))
    assert report['rows'][0]['inlier_observations']==3


def test_nonmonotonic_graph_is_rejected_instead_of_temporal_mismatching():
    rows,graph,original=inputs()
    with pytest.raises(ValueError,match='increasing'):finalize(rows,graph[::-1],original)


def test_csv_reader_sorts_and_keeps_latest_duplicate_pose(tmp_path):
    path=tmp_path/'poses.csv'
    path.write_text('node_id,stamp_sim_s,odom_x,odom_y,odom_yaw,map_x,map_y,map_yaw\n'
        '2,2,0,0,0,1,0,0\n1,1,0,0,0,0,0,0\n3,2,0,0,0,2,0,0\n')
    values=read_graph(path)
    assert values[:,0].tolist()==[1,3] and values[:,5].tolist()==[0,2]


@pytest.mark.parametrize('valid_export',[True,False])
def test_final_csv_update_preserves_original_on_failed_export_and_keeps_evidence_separate(tmp_path,valid_export):
    rows,graph,original=inputs();(tmp_path/'map.data').write_bytes(b'fixture')
    before='id,x,y\n7,-10,-10\n';(tmp_path/'markers.csv').write_text(before)
    exporter=tmp_path/'fixture_exporter.py'
    text='node_id,stamp_sim_s,odom_x,odom_y,odom_yaw,map_x,map_y,map_yaw\n'
    for values in graph:text+=','.join(map(str,values))+'\n'
    exporter.write_text('#!/usr/bin/python3\nimport pathlib,sys\n'+
        ('pathlib.Path(sys.argv[2]).write_text('+repr(text)+')\n' if valid_export else 'sys.exit(1)\n'))
    exporter.chmod(0o755)
    result,report=finalize_output(rows,original,tmp_path,exporter)
    if valid_export:
        assert result[7]==pytest.approx((2,5)) and report['applied']
        assert (tmp_path/'markers_before_graph.csv').read_text()==before
        with (tmp_path/'markers.csv').open() as stream:stored=list(csv.DictReader(stream))
        assert len(stored)==1 and float(stored[0]['y'])==pytest.approx(5)
    else:
        assert result==original and not report['applied']
        assert (tmp_path/'markers.csv').read_text()==before
    assert not json.loads((tmp_path/'graph_landmark_update.json').read_text())['driving_time_and_approach_credits_modified']
