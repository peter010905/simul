"""Observed sensor and motion timing for the real Gazebo benchmark."""

import csv
import math
from pathlib import Path

import numpy as np


class Rate:
    def __init__(self):
        self.samples=0;self.first=None;self.last=None;self.gaps=[];self.duplicates=0;self.resets=0

    def add(self,stamp,wall):
        if not math.isfinite(stamp) or not math.isfinite(wall):return
        self.samples+=1
        if self.first is None:self.first=(stamp,wall)
        if self.last is not None:
            delta=stamp-self.last[0]
            if delta>0:self.gaps.append(delta)
            elif delta==0:self.duplicates+=1
            else:self.resets+=1
        self.last=(stamp,wall)

    def summary(self):
        wall_span=self.last[1]-self.first[1] if self.first else 0.
        sim_span=self.last[0]-self.first[0] if self.first else 0.
        return dict(samples=self.samples,sim_hz=(len(self.gaps)/sim_span if sim_span>0 and not self.resets else None),
                    wall_hz=((self.samples-1)/wall_span if wall_span>0 else None),
                    gap_sim_p95=(float(np.quantile(self.gaps,.95)) if self.gaps else None),
                    gap_sim_max=(float(max(self.gaps)) if self.gaps else None),
                    duplicate_stamps=self.duplicates,clock_resets=self.resets)


class RuntimeMetrics:
    def __init__(self):
        self.started=None;self.finished=False;self.rates={name:Rate() for name in ('camera','scan','odom','command','clock')}
        self.trace=[];self.last_saved={};self.turn_speeds=[];self.forward_speeds=[]
        self.latest_clock=None;self.camera_lags=[]
        self.scan_valid_fractions=[];self.scan_sector_fractions=[]

    def start(self,wall):
        if self.started is None:self.started=wall

    def stop(self):
        if self.started is not None:self.finished=True

    def add_scan(self,stamp,wall,ranges,range_min,range_max):
        if self.started is None or self.finished:return
        self.add('scan',stamp,wall)
        values=np.asarray(ranges,dtype=float)
        if not len(values):return
        valid=np.isfinite(values)&(values>=range_min)&(values<range_max)
        self.scan_valid_fractions.append(float(valid.mean()))
        if len(values)>=8:
            self.scan_sector_fractions.append([float(v.mean()) for v in np.array_split(valid,8)])

    def add(self,kind,stamp,wall,linear=None,angular=None):
        if self.started is None or self.finished:return
        values=[stamp,wall]+([] if linear is None else [linear,angular])
        if not all(math.isfinite(v) for v in values):return
        self.rates[kind].add(stamp,wall)
        if kind=='clock':
            self.latest_clock=stamp
            return
        if kind=='camera' and self.latest_clock is not None and self.latest_clock>=stamp:
            self.camera_lags.append(self.latest_clock-stamp)
        if kind=='odom' and linear is not None:
            if linear<.03 and abs(angular)>.05:self.turn_speeds.append(abs(angular))
            if linear>.03:self.forward_speeds.append(linear)
        if stamp-self.last_saved.get(kind,-float('inf'))>=.1-1e-9:
            self.trace.append((kind,stamp,wall-self.started,linear,angular))
            self.last_saved[kind]=stamp

    def summary(self):
        clock=self.rates['clock']
        factor=None
        if clock.first and clock.last[1]>clock.first[1] and not clock.resets:
            factor=(clock.last[0]-clock.first[0])/(clock.last[1]-clock.first[1])
        def distribution(values):
            return dict(samples=len(values),median=float(np.median(values)) if values else None,
                        p95=float(np.quantile(values,.95)) if values else None)
        return dict(measured=any(rate.samples for rate in self.rates.values()),
                    complete_sensor_coverage=all(self.rates[k].samples>1 for k in ('camera','scan','odom','clock')),
                    complete_sensor_coverage_note='Required streams observed; stream presence does not prove sensor correctness',
                    streams={name:rate.summary() for name,rate in self.rates.items()},
                    real_time_factor=factor,stationary_turn_rad_s=distribution(self.turn_speeds),
                    moving_speed_m_s=distribution(self.forward_speeds),
                    processed_camera_lag_sim_s=distribution(self.camera_lags),
                    lidar_valid_beam_fraction=distribution(self.scan_valid_fractions),
                    lidar_sector_valid_fractions=np.mean(self.scan_sector_fractions,axis=0).tolist() if self.scan_sector_fractions else None,
                    lidar_sector_frame='Eight equal angular sectors in LaserScan sensor frame',
                    note='Observed after mission start; camera lag uses the most recently received clock, not a synchronized latency measurement')

    def save_trace(self,path):
        with Path(path).open('w',newline='') as stream:
            writer=csv.writer(stream)
            writer.writerow(['stream','stamp_sim_seconds','receipt_wall_seconds_since_start','linear_speed_m_s','angular_speed_rad_s'])
            writer.writerows(self.trace)
