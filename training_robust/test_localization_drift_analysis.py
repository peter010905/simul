import csv
import json
import math

import pytest

from analyze_localization_drift import analyze


@pytest.mark.parametrize('relative_sensor_error',[0.,.6])
def test_counterfactual_removes_only_robot_pose_error_not_relative_sensor_error(tmp_path,relative_sensor_error):
    scene=tmp_path/'scene.yaml';scene.write_text('targets:\n- {id: 7, x: 3.0, y: 0.0}\n')
    (tmp_path/'evaluation.json').write_text(json.dumps(dict(scenario=str(scene),success=False,
        elapsed_seconds=3.,ground_truth_audit=dict(world_to_map=[0,0,0],position_errors_m={'7':1.}))))
    angle=math.pi/6;mx,my=.8,.3;distance=2.+relative_sensor_error
    point=[mx+math.cos(angle)*distance,my+math.sin(angle)*distance,0.]
    (tmp_path/'marker_observations.json').write_text(json.dumps([dict(id=7,stamp=2.,map_point=point)]))
    with (tmp_path/'pose_trace.csv').open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['gt_stamp','gt_x','gt_y','gt_yaw','map_stamp',
            'map_x','map_y','map_yaw','odom_stamp','odom_x','odom_y','odom_yaw'])
        for stamp in [1,2,3]:writer.writerow([stamp,1,0,0,stamp,mx,my,angle,stamp,1,0,0])
    fine=tmp_path/'evaluator_gt_fine_test.csv'
    fine.write_text('sim_seconds,world_x,world_y,world_yaw\n1,1,0,0\n2,1,0,0\n3,1,0,0\n')
    fine.with_suffix('.json').write_text(json.dumps(dict(complete=True)))
    result=analyze(tmp_path)['by_id']['7']
    assert result['counterfactual_pose_corrected_marker_error_m']['median']==pytest.approx(relative_sensor_error)
    assert result['recorded_map_marker_error_m']['median']>.5
