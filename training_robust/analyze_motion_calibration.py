"""Compare raw wheel and EKF velocities to evaluator-only world motion on frozen CSV snapshots."""
import argparse
from collections import Counter
import csv
import io
import json
from pathlib import Path

import numpy as np

from continual_native_curriculum import digest,save


def extract(path):
    rows=list(csv.DictReader(io.StringIO(path.read_text())));streams={}
    for kind in ['ground_truth','wheel','ekf']:
        keys=['stamp_sim_s','x','y','yaw'] if kind=='ground_truth' else ['stamp_sim_s','linear_x']
        a=np.array([[float(row[k]) for k in keys] for row in rows if row['stream']==kind])
        if len(a)<2:raise ValueError(f'Insufficient {kind} samples')
        a=a[np.argsort(a[:,0],kind='stable')];streams[kind]=a[np.r_[np.diff(a[:,0])>0,True]]
    truth=streams['ground_truth'];truth[:,3]=np.unwrap(truth[:,3]);windows=[];rejected=Counter()
    start=np.ceil(max(a[0,0] for a in streams.values()));end=np.floor(min(a[-1,0] for a in streams.values()))
    for when in np.arange(start,end-1+1e-9,1.):
        covered=True
        for key,a in streams.items():
            lo=max(0,np.searchsorted(a[:,0],when)-1);hi=min(len(a),np.searchsorted(a[:,0],when+1)+1)
            if np.diff(a[lo:hi,0]).max(initial=0)>(.08 if key=='ground_truth' else .10):covered=False
        if not covered:rejected['sampling_gap']+=1;continue
        gt=truth[(truth[:,0]>=when)&(truth[:,0]<=when+1)]
        if len(gt)<2 or np.ptp(gt[:,3])>.15:rejected['heading_change_above_0_15rad']+=1;continue
        x=np.interp([when,when+1],truth[:,0],truth[:,1]);y=np.interp([when,when+1],truth[:,0],truth[:,2])
        heading=np.mean(gt[:,3]);actual=(x[1]-x[0])*np.cos(heading)+(y[1]-y[0])*np.sin(heading)
        if not .12<actual<.7:rejected['actual_forward_speed_outside_0_12_to_0_7']+=1;continue
        times=np.linspace(when,when+1,51)
        measured={key:np.interp(times,streams[key][:,0],streams[key][:,1]) for key in ['wheel','ekf']}
        windows.append(dict(start_sim_s=float(when),actual_forward_m_s=float(actual),
            wheel_forward_m_s=float(measured['wheel'].mean()),ekf_forward_m_s=float(measured['ekf'].mean()),
            wheel_within_window_std_m_s=float(measured['wheel'].std())))
    return dict(source=str(path),source_sha256=digest(path),windows=windows,rejected=dict(rejected))


def evaluate(rows,scale):
    if not rows:raise ValueError('No eligible motion windows')
    actual=np.array([r['actual_forward_m_s'] for r in rows]);raw=np.array([r['wheel_forward_m_s'] for r in rows])
    filtered=np.array([r['ekf_forward_m_s'] for r in rows])
    return dict(windows=len(rows),median_raw_to_actual_ratio=float(np.median(raw/actual)),
        median_ekf_to_actual_ratio=float(np.median(filtered/actual)),
        raw_rmse_m_s=float(np.sqrt(np.mean((raw-actual)**2))),
        scaled_raw_rmse_m_s=float(np.sqrt(np.mean((raw*scale-actual)**2))),
        scaled_median_relative_error=float(np.median(raw*scale/actual-1)),
        scaled_p90_absolute_relative_error=float(np.quantile(abs(raw*scale/actual-1),.9)))


def analyze(training,validation):
    train=extract(training);valid=extract(validation)
    if len(train['windows'])<10 or len(valid['windows'])<10:
        raise ValueError('Require at least10 eligible windows in each frozen capture')
    raw=np.array([r['wheel_forward_m_s'] for r in train['windows']]);actual=np.array([r['actual_forward_m_s'] for r in train['windows']])
    scale=float(raw@actual/(raw@raw))
    return dict(evidence_type='offline_velocity_calibration_from_two_frozen_native_sensor_capture_segments',
        fitted_forward_velocity_scale=scale,rounded_candidate_scale=round(scale,2),
        training=evaluate(train['windows'],round(scale,2)),validation=evaluate(valid['windows'],round(scale,2)),
        captures=[train,valid],policy_or_runtime_changed=False,ground_truth_only_in_evaluator=True,
        native_improvement_verified=False,limits=[
            'Two captured trajectory segments;not a full-mission generalization or hardware calibration result',
            'World displacement over1simsecond is projected onto mean heading;windows above0.15rad heading change or with sampling gaps are rejected',
            'Within-window raw velocity variation includes real acceleration and cannot alone identify measurement covariance',
            'A fitted fixed scale is a candidate for new native testing,not permission to feed ground truth to a running estimator'])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['training','validation','output']:parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();save(args.output,analyze(args.training.resolve(),args.validation.resolve()))
