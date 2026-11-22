"""Synthetic telemetry verifies calculations, not ROS transport or Gazebo speed."""

import csv
import pytest

from osp_mission.runtime_metrics import Rate,RuntimeMetrics


def test_missing_data_is_not_reported_as_measured_or_as_zero_velocity():
    metrics=RuntimeMetrics()
    metrics.add('camera',1.,10.)
    report=metrics.summary()
    assert not report['measured'] and not report['complete_sensor_coverage']
    assert report['stationary_turn_rad_s']['median'] is None
    assert report['real_time_factor'] is None


def test_simulated_and_wall_rates_are_distinct_and_initialization_is_excluded(tmp_path):
    metrics=RuntimeMetrics();metrics.add('odom',0.,0.,0.,3.)
    metrics.start(10.)
    for i in range(6):
        stamp=i*.2;wall=10.+i*2.
        metrics.add('clock',stamp,wall)
        metrics.add('camera',stamp,wall+.01)
        metrics.add_scan(stamp,wall+.015,[2.]*640,.164,12.)
        metrics.add('odom',stamp,wall+.02,0.,.8)
        metrics.add('command',stamp,wall+.03,0.,1.)
    metrics.stop();metrics.add('odom',2.,40.,0.,3.)
    report=metrics.summary()
    assert report['measured'] and report['complete_sensor_coverage']
    assert report['streams']['camera']['sim_hz']==pytest.approx(5.)
    assert report['streams']['camera']['wall_hz']==pytest.approx(.5)
    assert report['real_time_factor']==pytest.approx(.1)
    assert report['stationary_turn_rad_s']['median']==pytest.approx(.8)
    destination=tmp_path/'trace.csv';metrics.save_trace(destination)
    rows=list(csv.DictReader(destination.open()))
    assert len(rows)==24 and all(float(row['receipt_wall_seconds_since_start'])>=0 for row in rows)
    assert report['lidar_valid_beam_fraction']['median']==1.


def test_receiving_scan_messages_does_not_hide_blind_lidar_sectors():
    metrics=RuntimeMetrics();metrics.start(0.)
    for stamp in [0.,.2]:
        for kind in ['camera','odom','clock']:metrics.add(kind,stamp,stamp)
        metrics.add_scan(stamp,stamp,[float('inf')]*480+[2.]*160,.164,12.)
    report=metrics.summary()
    assert report['complete_sensor_coverage']
    assert report['lidar_valid_beam_fraction']['median']==.25
    assert report['lidar_sector_valid_fractions']==[0.]*6+[1.]*2


def test_clock_reset_invalidates_a_single_sim_rate():
    rate=Rate();rate.add(2.,1.);rate.add(3.,2.);rate.add(0.,3.)
    assert rate.summary()['clock_resets']==1
    assert rate.summary()['sim_hz'] is None
